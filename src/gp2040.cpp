// GP2040 includes
#include "gp2040.h"
#include "helper.h"
#include "system.h"
#include "enums.pb.h"

#include "build_info.h"
#include "peripheralmanager.h"
#include "storagemanager.h"
#include "addonmanager.h"
#include "types.h"
#include "usbhostmanager.h"

// Inputs for Core0
#include "addons/analog.h"
#include "addons/bootsel_button.h"
#include "addons/focus_mode.h"
#include "addons/dualdirectional.h"
#include "addons/tilt.h"
#include "addons/keyboard_host.h"
#include "addons/i2canalog1219.h"
#include "addons/reverse.h"
#include "addons/turbo.h"
#include "addons/slider_socd.h"
#include "addons/spi_analog_ads1256.h"
#include "addons/wiiext.h"
#include "addons/input_macro.h"
#include "addons/snes_input.h"
#include "addons/rotaryencoder.h"
#include "addons/i2c_gpio_pcf8575.h"
#include "addons/gamepad_usb_host.h"
#include "addons/he_trigger.h"
#include "addons/tg16_input.h"

// Pico includes
#include "pico/bootrom.h"
#include "pico/time.h"
#include "hardware/adc.h"

#include "rndis.h"

// TinyUSB
#include "tusb.h"

// USB Input Class Drivers
#include "drivermanager.h"

// --- 拟人化逻辑核心 ---
#include <cmath>
#include <cstdlib>

class StickHumanizer {
private:
    float curX = 32767.5f, curY = 32767.5f;

    // 参数配置
    const float smoothFactor;    // 平滑度 (越小越平滑)
    const float jitterRange;     // 随机抖动幅度
    const float responseCurve;   // 响应曲线 (1.0线性, >1.0指数)
    const uint16_t deadzone = 800;

public:
    StickHumanizer(float s, float j, float c) 
        : smoothFactor(s), jitterRange(j), responseCurve(c) {}

    void apply(uint16_t& rawX, uint16_t& rawY) {
        float targetX = (float)rawX;
        float targetY = (float)rawY;

        // --- 针对 Issue #1515 的修正：检测键盘极值 ---
        // 如果是按键产生的 0, 32767, 65535，我们适当加速平滑，防止被微小的鼠标漂移信号锁定
        float currentSmooth = smoothFactor;
        bool isKeyInput = (rawX == 0 || rawX == 32767 || rawX == 65535) && 
                           (rawY == 0 || rawY == 32767 || rawY == 65535);
        if (isKeyInput) currentSmooth *= 1.25f; 

        // 1. 响应曲线处理 (增强 FPS 微调感)
        if (responseCurve > 1.01f) {
            float normX = (targetX - 32767.5f) / 32767.5f;
            float normY = (targetY - 32767.5f) / 32767.5f;
            auto applyCurve = [&](float val) {
                float sign = (val > 0) ? 1.0f : -1.0f;
                return std::pow(std::abs(val), responseCurve) * sign;
            };
            targetX = applyCurve(normX) * 32767.5f + 32767.5f;
            targetY = applyCurve(normY) * 32767.5f + 32767.5f;
        }

        // 2. 圆形限域 (解决对角线 1.414 倍速问题)
        float nX = (targetX - 32767.5f) / 32767.5f;
        float nY = (targetY - 32767.5f) / 32767.5f;
        float mag = std::sqrt(nX*nX + nY*nY);
        if (mag > 1.0f) {
            targetX = (nX / mag) * 32767.5f + 32767.5f;
            targetY = (nY / mag) * 32767.5f + 32767.5f;
        }

        // 3. 指数平滑 (消除键盘瞬发的“方波”信号)
        curX += (targetX - curX) * currentSmooth;
        curY += (targetY - curY) * currentSmooth;

        // 4. 拟人抖动与死区判定
        if (std::abs(curX - 32767.5f) > deadzone || std::abs(curY - 32767.5f) > deadzone) {
            rawX = (uint16_t)(curX + ((std::rand() % 100) / 100.0f - 0.5f) * jitterRange);
            rawY = (uint16_t)(curY + ((std::rand() % 100) / 100.0f - 0.5f) * jitterRange);
        } else {
            rawX = 32767;
            rawY = 32767;
        }
    }
};

// 实例化：左摇杆平滑移动，右摇杆 1.8 倍曲线吸附
static StickHumanizer leftHumanizer(0.15f, 250.0f, 1.0f); 
static StickHumanizer rightHumanizer(0.25f, 200.0f, 1.8f);

void GP2040::setup() {
	Storage::getInstance().init();

    // 初始化随机种子
	srand(get_absolute_time()._private_us);

	PeripheralManager::getInstance().initUSB();
	if ( PeripheralManager::getInstance().isUSBEnabled(0) ) {
		set_sys_clock_khz(120000, true); 
	}

	PeripheralManager::getInstance().initSPI();
	PeripheralManager::getInstance().initI2C();

	Gamepad * gamepad = new Gamepad();
	Gamepad * processedGamepad = new Gamepad();
	Storage::getInstance().SetGamepad(gamepad);
	Storage::getInstance().SetProcessedGamepad(processedGamepad);
	Storage::getInstance().setFunctionalPinMappings();

	gamepad->auxState.power.pluggedIn = true;
	gamepad->auxState.power.charging = false;
	gamepad->auxState.power.level = GAMEPAD_AUX_MAX_POWER;
	gamepad->setup();
	gamepad->lastReinitProfileNumber = Storage::getInstance().getGamepadOptions().profileNumber;

	this->initializeStandardGpio();

	const GamepadOptions& gamepadOptions = Storage::getInstance().getGamepadOptions();
	bootActions.insert({GAMEPAD_MASK_B1, gamepadOptions.inputModeB1});
	bootActions.insert({GAMEPAD_MASK_B2, gamepadOptions.inputModeB2});
	bootActions.insert({GAMEPAD_MASK_B3, gamepadOptions.inputModeB3});
	bootActions.insert({GAMEPAD_MASK_B4, gamepadOptions.inputModeB4});
	bootActions.insert({GAMEPAD_MASK_L1, gamepadOptions.inputModeL1});
	bootActions.insert({GAMEPAD_MASK_L2, gamepadOptions.inputModeL2});
	bootActions.insert({GAMEPAD_MASK_R1, gamepadOptions.inputModeR1});
	bootActions.insert({GAMEPAD_MASK_R2, gamepadOptions.inputModeR2});

	adc_init();

	addons.LoadUSBAddon(new KeyboardHostAddon());
	addons.LoadUSBAddon(new GamepadUSBHostAddon());
	addons.LoadAddon(new AnalogInput());
	addons.LoadAddon(new HETriggerAddon());
	addons.LoadAddon(new BootselButtonAddon());
	addons.LoadAddon(new DualDirectionalInput());
	addons.LoadAddon(new FocusModeAddon());
	addons.LoadAddon(new I2CAnalog1219Input());
	addons.LoadAddon(new SPIAnalog1256Input());
	addons.LoadAddon(new WiiExtensionInput());
	addons.LoadAddon(new SNESpadInput());
	addons.LoadAddon(new SliderSOCDInput());
	addons.LoadAddon(new TiltInput());
	addons.LoadAddon(new RotaryEncoderInput());
	addons.LoadAddon(new PCF8575Addon());
	addons.LoadAddon(new TG16padInput());
	addons.LoadAddon(new ReverseInput());
	addons.LoadAddon(new TurboInput());
	addons.LoadAddon(new InputMacro());

	InputMode inputMode = gamepad->getOptions().inputMode;
	const BootAction bootAction = getBootAction();
	switch (bootAction) {
		case BootAction::ENTER_WEBCONFIG_MODE: inputMode = INPUT_MODE_CONFIG; break;
		case BootAction::ENTER_USB_MODE: reset_usb_boot(0, 0); return;
		case BootAction::SET_INPUT_MODE_SWITCH: inputMode = INPUT_MODE_SWITCH; break;
		case BootAction::SET_INPUT_MODE_KEYBOARD: inputMode = INPUT_MODE_KEYBOARD; break;
		case BootAction::SET_INPUT_MODE_XINPUT: inputMode = INPUT_MODE_XINPUT; break;
		case BootAction::SET_INPUT_MODE_PS4: inputMode = INPUT_MODE_PS4; break;
		case BootAction::SET_INPUT_MODE_XBONE: inputMode = INPUT_MODE_XBONE; break;
		case BootAction::NONE:
		default: break;
	}

	DriverManager::getInstance().setup(inputMode);
	if (inputMode != INPUT_MODE_CONFIG && inputMode != gamepad->getOptions().inputMode) {
		gamepad->setInputMode(inputMode);
		Storage::getInstance().save(true);
	}

	EventManager::getInstance().registerEventHandler(GP_EVENT_STORAGE_SAVE, GPEVENT_CALLBACK(this->handleStorageSave(event)));
	EventManager::getInstance().registerEventHandler(GP_EVENT_RESTART, GPEVENT_CALLBACK(this->handleSystemReboot(event)));
}

void GP2040::initializeStandardGpio() {
	GpioMappingInfo* pinMappings = Storage::getInstance().getProfilePinMappings();
	buttonGpios = 0;
	for (Pin_t pin = 0; pin < (Pin_t)NUM_BANK0_GPIOS; pin++) {
		if (pinMappings[pin].action > 0) {
			gpio_init(pin);
			gpio_set_dir(pin, GPIO_IN);
			gpio_pull_up(pin);
			buttonGpios |= 1 << pin;
		}
	}
}

void GP2040::deinitializeStandardGpio() {
	GpioMappingInfo* pinMappings = Storage::getInstance().getProfilePinMappings();
	for (Pin_t pin = 0; pin < (Pin_t)NUM_BANK0_GPIOS; pin++) {
		if (pinMappings[pin].action > 0) { gpio_deinit(pin); }
	}
}

void GP2040::debounceGpioGetAll() {
	Mask_t raw_gpio = ~gpio_get_all();
	Gamepad* gamepad = Storage::getInstance().GetGamepad();
	if (gamepad->debouncedGpio == (raw_gpio & buttonGpios)) return;
	uint32_t debounceDelay = Storage::getInstance().getGamepadOptions().debounceDelay;
	if (debounceDelay == 0) {
		gamepad->debouncedGpio = raw_gpio;
		return;
	}
	uint32_t now = getMillis();
	for (Pin_t pin = 0; pin < (Pin_t)NUM_BANK0_GPIOS; pin++) {
		Mask_t pin_mask = 1 << pin;
		if (buttonGpios & pin_mask) {
			if ((gamepad->debouncedGpio & pin_mask) != (raw_gpio & pin_mask) && ((now - gpioDebounceTime[pin]) > debounceDelay)) {
				gamepad->debouncedGpio ^= pin_mask;
				gpioDebounceTime[pin] = now;
			}
		}
	}
}

void GP2040::run() {
	bool configMode = DriverManager::getInstance().isConfigMode();
	GPDriver * inputDriver = DriverManager::getInstance().getDriver();
	Gamepad * gamepad = Storage::getInstance().GetGamepad();
	Gamepad * processedGamepad = Storage::getInstance().GetProcessedGamepad();
	GamepadState prevState;

	tud_init(TUD_OPT_RHPORT);
	USBHostManager::getInstance().start();
	if (configMode == true ) { rndis_init(); }

	while (1) {
		this->getReinitGamepad(gamepad);
		memcpy(&prevState, &gamepad->state, sizeof(GamepadState));
		debounceGpioGetAll();
		gamepad->read();
		checkRawState(prevState, gamepad->state);
		USBHostManager::getInstance().process();

		if (configMode == true) {
			inputDriver->process(gamepad);
			rebootHotkeys.process(gamepad, configMode);
			checkSaveRebootState();
			continue;
		}

		addons.PreprocessAddons();
		gamepad->hotkey();
		rebootHotkeys.process(gamepad, configMode);
		gamepad->process();
		addons.ProcessAddons();

        // --- 拟人化执行位置 ---
		if (configMode == false) {
            leftHumanizer.apply(gamepad->state.lx, gamepad->state.ly);
            rightHumanizer.apply(gamepad->state.rx, gamepad->state.ry);
        }

		checkProcessedState(processedGamepad->state, gamepad->state);
		memcpy(&processedGamepad->state, &gamepad->state, sizeof(GamepadState));

		bool processed = inputDriver->process(gamepad);
		tud_task();
		addons.PostprocessAddons(processed);
		checkSaveRebootState();
	}
}

void GP2040::getReinitGamepad(Gamepad * gamepad) {
	GamepadOptions& gamepadOptions = Storage::getInstance().getGamepadOptions();
	if (gamepad->lastReinitProfileNumber != gamepadOptions.profileNumber) {
		uint32_t previousProfile = gamepad->lastReinitProfileNumber;
		uint32_t currentProfile = gamepadOptions.profileNumber;
		this->deinitializeStandardGpio();
		Storage::getInstance().setFunctionalPinMappings();
		this->initializeStandardGpio();
		gamepad->reinit();
		addons.ReinitializeAddons();
		gamepad->lastReinitProfileNumber = currentProfile;
		EventManager::getInstance().triggerEvent(new GPProfileChangeEvent(previousProfile, currentProfile));
	}
}

GP2040::BootAction GP2040::getBootAction() {
	switch (System::takeBootMode()) {
		case System::BootMode::GAMEPAD: return BootAction::NONE;
		case System::BootMode::WEBCONFIG: return BootAction::ENTER_WEBCONFIG_MODE;
		case System::BootMode::USB: return BootAction::ENTER_USB_MODE;
		case System::BootMode::DEFAULT:
			{
				Gamepad * gamepad = Storage::getInstance().GetGamepad();
				Gamepad * processedGamepad = Storage::getInstance().GetProcessedGamepad();
				debounceGpioGetAll();
				gamepad->read();
				addons.PreprocessAddons();
				gamepad->process();
				addons.ProcessAddons();
				memcpy(&processedGamepad->state, &gamepad->state, sizeof(GamepadState));

				const ForcedSetupOptions& forcedSetupOptions = Storage::getInstance().getForcedSetupOptions();
				bool webConfigLocked = forcedSetupOptions.mode == FORCED_SETUP_MODE_LOCK_WEB_CONFIG || forcedSetupOptions.mode == FORCED_SETUP_MODE_LOCK_BOTH;

				if (gamepad->pressedS1() && gamepad->pressedS2() && gamepad->pressedUp()) {
					return BootAction::ENTER_USB_MODE;
				} else if (!webConfigLocked && gamepad->pressedS2()) {
					return BootAction::ENTER_WEBCONFIG_MODE;
				}
				break;
			}
	}
	return BootAction::NONE;
}

GP2040::RebootHotkeys::RebootHotkeys() : active(false), noButtonsPressedTimeout(nil_time), webConfigHotkeyMask(GAMEPAD_MASK_S2 | GAMEPAD_MASK_B3 | GAMEPAD_MASK_B4), bootselHotkeyMask(GAMEPAD_MASK_S1 | GAMEPAD_MASK_B3 | GAMEPAD_MASK_B4), rebootHotkeysHoldTimeout(nil_time) {}

void GP2040::RebootHotkeys::process(Gamepad* gamepad, bool configMode) {
	if (!active) {
		if (gamepad->state.buttons == 0) {
			if (is_nil_time(noButtonsPressedTimeout)) { noButtonsPressedTimeout = make_timeout_time_us(REBOOT_HOTKEY_ACTIVATION_TIME_MS); }
			if (time_reached(noButtonsPressedTimeout)) { active = true; }
		} else { noButtonsPressedTimeout = nil_time; }
	} else {
		if (gamepad->state.buttons == webConfigHotkeyMask || gamepad->state.buttons == bootselHotkeyMask) {
			if (is_nil_time(rebootHotkeysHoldTimeout)) { rebootHotkeysHoldTimeout = make_timeout_time_ms(REBOOT_HOTKEY_HOLD_TIME_MS); }
			if (time_reached(rebootHotkeysHoldTimeout)) {
				if (gamepad->state.buttons == webConfigHotkeyMask) { System::reboot(configMode ? System::BootMode::GAMEPAD : System::BootMode::WEBCONFIG); }
				else if (gamepad->state.buttons == bootselHotkeyMask) { System::reboot(System::BootMode::USB); }
			}
		} else { rebootHotkeysHoldTimeout = nil_time; }
	}
}

void GP2040::checkRawState(GamepadState prevState, GamepadState currState) {
    if (((currState.aux & ~prevState.aux) != 0) || ((currState.dpad & ~prevState.dpad) != 0) || ((currState.buttons & ~prevState.buttons) != 0)) {
        EventManager::getInstance().triggerEvent(new GPButtonDownEvent((currState.dpad & ~prevState.dpad), (currState.buttons & ~prevState.buttons), (currState.aux & ~prevState.aux)));
    }
    if (((prevState.aux & ~currState.aux) != 0) || ((prevState.dpad & ~currState.dpad) != 0) || ((prevState.buttons & ~currState.buttons) != 0)) {
        EventManager::getInstance().triggerEvent(new GPButtonUpEvent((prevState.dpad & ~currState.dpad), (prevState.buttons & ~currState.buttons), (prevState.aux & ~currState.aux)));
    }
}

void GP2040::checkProcessedState(GamepadState prevState, GamepadState currState) {
    if (((currState.aux & ~prevState.aux) != 0) || ((currState.dpad & ~prevState.dpad) != 0) || ((currState.buttons & ~prevState.buttons) != 0)) {
        EventManager::getInstance().triggerEvent(new GPButtonProcessedDownEvent((currState.dpad & ~prevState.dpad), (currState.buttons & ~prevState.buttons), (currState.aux & ~prevState.aux)));
    }
    if (((prevState.aux & ~currState.aux) != 0) || ((prevState.dpad & ~currState.dpad) != 0) || ((prevState.buttons & ~currState.buttons) != 0)) {
        EventManager::getInstance().triggerEvent(new GPButtonProcessedUpEvent((prevState.dpad & ~currState.dpad), (prevState.buttons & ~currState.buttons), (prevState.aux & ~currState.aux)));
    }
    if ((currState.lx != prevState.lx) || (currState.ly != prevState.ly) || (currState.rx != prevState.rx) || (currState.ry != prevState.ry) || (currState.lt != prevState.lt) || (currState.rt != prevState.rt)) {
        EventManager::getInstance().triggerEvent(new GPAnalogProcessedMoveEvent(currState.lx, currState.ly, currState.rx, currState.ry, currState.lt, currState.rt));
    }
}

void GP2040::checkSaveRebootState() {
	if (saveRequested) {
		saveRequested = false;
		Storage::getInstance().save(forceSave);
	}

	if (rebootRequested) {
		rebootRequested = false;
        // 直接执行重启，不再等待微小的延迟，这样就不会用到那些报错的变量
		System::reboot(rebootMode);
	}
}

void GP2040::handleStorageSave(GPEvent* e) {
	saveRequested = true;
	forceSave = ((GPStorageSaveEvent*)e)->forceSave;
	rebootRequested = ((GPStorageSaveEvent*)e)->restartAfterSave;
	rebootMode = System::BootMode::DEFAULT;
}

void GP2040::handleSystemReboot(GPEvent* e) {
	rebootRequested = true;
	rebootMode = ((GPRestartEvent*)e)->bootMode;
}
