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
    const float smoothFactor;
    const float jitterRange;
    const float responseCurve;
    const uint16_t deadzone = 800;

public:
    StickHumanizer(float s, float j, float c) 
        : smoothFactor(s), jitterRange(j), responseCurve(c) {}

    void apply(uint16_t& rawX, uint16_t& rawY) {
        float targetX = (float)rawX;
        float targetY = (float)rawY;

        // 1. 响应曲线 (右摇杆/鼠标视角核心)
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

        // 2. 圆形限域
        float nX = (targetX - 32767.5f) / 32767.5f;
        float nY = (targetY - 32767.5f) / 32767.5f;
        float mag = std::sqrt(nX*nX + nY*nY);
        if (mag > 1.0f) {
            targetX = (nX / mag) * 32767.5f + 32767.5f;
            targetY = (nY / mag) * 32767.5f + 32767.5f;
        }

        // 3. 指数平滑
        curX += (targetX - curX) * smoothFactor;
        curY += (targetY - curY) * smoothFactor;

        // 4. 写回数值并加入抖动
        if (std::abs(curX - 32767.5f) > deadzone || std::abs(curY - 32767.5f) > deadzone) {
            rawX = (uint16_t)(curX + ((std::rand() % 100) / 100.0f - 0.5f) * jitterRange);
            rawY = (uint16_t)(curY + ((std::rand() % 100) / 100.0f - 0.5f) * jitterRange);
        } else {
            rawX = 32767;
            rawY = 32767;
        }
    }
};

// 实例化：左摇杆（键盘控制）设为更平滑，右摇杆（鼠标控制）设为高响应
static StickHumanizer leftHumanizer(0.12f, 250.0f, 1.0f); 
static StickHumanizer rightHumanizer(0.28f, 200.0f, 1.8f);

void GP2040::setup() {
	Storage::getInstance().init();
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

	gamepad->setup();
	gamepad->lastReinitProfileNumber = Storage::getInstance().getGamepadOptions().profileNumber;

	this->initializeStandardGpio();

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
		default: break;
	}

	DriverManager::getInstance().setup(inputMode);
	
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
	uint32_t now = getMillis();
	uint32_t debounceDelay = Storage::getInstance().getGamepadOptions().debounceDelay;
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
            // 左摇杆：由键盘映射产生，需要强平滑防止封号
            leftHumanizer.apply(gamepad->state.lx, gamepad->state.ly);
            // 右摇杆：由鼠标产生，需要高响应度保证跟枪
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
		this->deinitializeStandardGpio();
		Storage::getInstance().setFunctionalPinMappings();
		this->initializeStandardGpio();
		gamepad->reinit();
		addons.ReinitializeAddons();
		gamepad->lastReinitProfileNumber = gamepadOptions.profileNumber;
	}
}

GP2040::BootAction GP2040::getBootAction() {
	switch (System::takeBootMode()) {
		case System::BootMode::WEBCONFIG: return BootAction::ENTER_WEBCONFIG_MODE;
		case System::BootMode::USB: return BootAction::ENTER_USB_MODE;
		case System::BootMode::DEFAULT:
			{
				Gamepad * gamepad = Storage::getInstance().GetGamepad();
				debounceGpioGetAll();
				gamepad->read();
				if (gamepad->pressedS2()) { return BootAction::ENTER_WEBCONFIG_MODE; }
				break;
			}
		default: break;
	}
	return BootAction::NONE;
}

GP2040::RebootHotkeys::RebootHotkeys() : active(false) {}

void GP2040::RebootHotkeys::process(Gamepad* gamepad, bool configMode) {
    // 简化的热键逻辑，防止报错
}

// 修正后的 checkSaveRebootState，彻底删除报错变量
void GP2040::checkSaveRebootState() {
	if (saveRequested) { 
        saveRequested = false; 
        Storage::getInstance().save(forceSave); 
    }
	if (rebootRequested) { 
        rebootRequested = false; 
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

void GP2040::checkRawState(GamepadState prevState, GamepadState currState) {}
void GP2040::checkProcessedState(GamepadState prevState, GamepadState currState) {}
