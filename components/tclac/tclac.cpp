#include "esphome.h"
#include "esphome/core/defines.h"
#include "tclac.h"

namespace esphome{
namespace tclac{

ClimateTraits tclacClimate::traits() {
    auto traits = climate::ClimateTraits();

    // 1. Apenas a temperatura atual
    traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
	
    // 2. Modos suportados. Mantemos AUTO como AUTO no Home Assistant.
    traits.set_supported_modes({
        climate::CLIMATE_MODE_OFF,
        climate::CLIMATE_MODE_AUTO,
        climate::CLIMATE_MODE_COOL,
        climate::CLIMATE_MODE_HEAT,
        climate::CLIMATE_MODE_DRY,
        climate::CLIMATE_MODE_FAN_ONLY
    });

    // 3. Ventilação (Ajustada: Diffuse = Turbo)
    traits.set_supported_fan_modes({
        climate::CLIMATE_FAN_AUTO,
        climate::CLIMATE_FAN_QUIET,
        climate::CLIMATE_FAN_LOW,
        climate::CLIMATE_FAN_MEDIUM,
        climate::CLIMATE_FAN_HIGH,
        climate::CLIMATE_FAN_DIFFUSE
    });

    // 4. Swing (Apenas Vertical)
    traits.set_supported_swing_modes({
        climate::CLIMATE_SWING_OFF,
        climate::CLIMATE_SWING_VERTICAL
    });

    // 5. Presets
    traits.set_supported_presets({
        climate::CLIMATE_PRESET_NONE,
        climate::CLIMATE_PRESET_ECO,
        climate::CLIMATE_PRESET_SLEEP,
        climate::CLIMATE_PRESET_COMFORT
    });

    return traits;
}

void tclacClimate::setup() {
    // === CORREÇÃO: Inicialização de estados sem usar .has_value() em Enums ===
    // mode e swing_mode são enums diretos, fan_mode é opcional
    if (!this->fan_mode.has_value()) {
        this->fan_mode = climate::CLIMATE_FAN_AUTO;
    }
    if (std::isnan(this->target_temperature)) {
        this->target_temperature = 18.0f;
    }
    this->publish_state();
    // ===================================================================

#ifdef CONF_RX_LED
	this->rx_led_pin_->setup();
	this->rx_led_pin_->digital_write(false);
#endif
#ifdef CONF_TX_LED
	this->tx_led_pin_->setup();
	this->tx_led_pin_->digital_write(false);
#endif
}

void tclacClimate::loop() {
	if (esphome::uart::UARTDevice::available() > 0) {
		// A linha está ocupada a receber; adiar transmissões de controlo.
		this->last_rx_ms_ = millis();
		dataShow(0, true);
		dataRX[0] = esphome::uart::UARTDevice::read();
		if (dataRX[0] != 0xBB) {
			dataShow(0,0);
			return;
		}

		// Mantemos os pequenos delays que já se mostraram estáveis no KUBO.
		delay(5);
		dataRX[1] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[2] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[3] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[4] = esphome::uart::UARTDevice::read();

		// Comprimentos válidos conhecidos: 61, 65 e 68 bytes totais.
		// Validar antes do read_array evita overflow se houver ruído na UART.
		if (dataRX[4] != 0x37 && dataRX[4] != 0x3B && dataRX[4] != 0x3E) {
			ESP_LOGW("TCL", "Bad frame length 0x%02X, dropped", dataRX[4]);
			while (esphome::uart::UARTDevice::available() > 0)
				esphome::uart::UARTDevice::read();
			dataShow(0,0);
			return;
		}

		if (!esphome::uart::UARTDevice::read_array(dataRX + 5, dataRX[4] + 1)) {
			ESP_LOGW("TCL", "Frame read timeout, dropped");
			dataShow(0,0);
			return;
		}

		this->last_rx_ms_ = millis();

		const size_t frame_size = static_cast<size_t>(dataRX[4]) + 6;
		byte check = getChecksum(dataRX, frame_size);
		if (check != dataRX[frame_size - 1]) {
			ESP_LOGD("TCL", "Invalid checksum %x", check);
			dataShow(0,0);
			return;
		}

		dataShow(0,0);
		readData();
	}
}

void tclacClimate::update() {
	tclacClimate::dataShow(1,1);
	this->esphome::uart::UARTDevice::write_array(poll, sizeof(poll));
	this->poll_sent_ms_ = millis();
	tclacClimate::dataShow(1,0);
}

void tclacClimate::readData() {
	
	current_temperature = float((( (dataRX[17] << 8) | dataRX[18] ) / 374 - 32)/1.8);
	target_temperature = (dataRX[FAN_SPEED_POS] & SET_TEMP_MASK) + 16;

	if (dataRX[MODE_POS] & ( 1 << 4)) {
		// O bit 0x20 pertence ao estado do display, não ao modo do AC.
		// Separá-lo impede que desligar o display faça COOL/HEAT parecer AUTO.
		this->display_status_ = (dataRX[MODE_POS] & DISPLAY_BIT) != 0;
		uint8_t modeswitch = MODE_MASK & dataRX[MODE_POS];
		uint8_t fanspeedswitch = FAN_SPEED_MASK & dataRX[FAN_SPEED_POS];
		uint8_t swingmodeswitch = SWING_MODE_MASK & dataRX[SWING_POS];

		switch (modeswitch) {
			case MODE_AUTO: mode = climate::CLIMATE_MODE_AUTO; break;
			case MODE_COOL: mode = climate::CLIMATE_MODE_COOL; break;
			case MODE_DRY: mode = climate::CLIMATE_MODE_DRY; break;
			case MODE_FAN_ONLY: mode = climate::CLIMATE_MODE_FAN_ONLY; break;
			case MODE_HEAT: mode = climate::CLIMATE_MODE_HEAT; break;
			default: mode = climate::CLIMATE_MODE_AUTO;
		}

		if ( dataRX[FAN_QUIET_POS] & FAN_QUIET) {
			fan_mode = climate::CLIMATE_FAN_QUIET;
		} else if (dataRX[MODE_POS] & FAN_DIFFUSE){
			fan_mode = climate::CLIMATE_FAN_DIFFUSE;
		} else {
			switch (fanspeedswitch) {
				case FAN_AUTO: fan_mode = climate::CLIMATE_FAN_AUTO; break;
				case FAN_LOW: fan_mode = climate::CLIMATE_FAN_LOW; break;
				case FAN_MIDDLE: fan_mode = climate::CLIMATE_FAN_MIDDLE; break;
				case FAN_MEDIUM: fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
				case FAN_HIGH: fan_mode = climate::CLIMATE_FAN_HIGH; break;
				case FAN_FOCUS: fan_mode = climate::CLIMATE_FAN_FOCUS; break;
				default: fan_mode = climate::CLIMATE_FAN_AUTO;
			}
		}

		switch (swingmodeswitch) {
			case SWING_OFF: swing_mode = climate::CLIMATE_SWING_OFF; break;
			case SWING_HORIZONTAL: swing_mode = climate::CLIMATE_SWING_HORIZONTAL; break;
			case SWING_VERTICAL: swing_mode = climate::CLIMATE_SWING_VERTICAL; break;
			case SWING_BOTH: swing_mode = climate::CLIMATE_SWING_BOTH; break;
		}
		
		preset = ClimatePreset::CLIMATE_PRESET_NONE;
		if (dataRX[7] & (1 << 6)){
			preset = ClimatePreset::CLIMATE_PRESET_ECO;
		} else if (dataRX[9] & (1 << 2)){
			preset = ClimatePreset::CLIMATE_PRESET_COMFORT;
		} else if (dataRX[19] & (1 << 0)){
			preset = ClimatePreset::CLIMATE_PRESET_SLEEP;
		}
		
	} else {
		mode = climate::CLIMATE_MODE_OFF;
		if (!fan_mode.has_value()) fan_mode = climate::CLIMATE_FAN_AUTO;
		swing_mode = climate::CLIMATE_SWING_OFF;
		preset = ClimatePreset::CLIMATE_PRESET_NONE;
	}
	this->publish_state();
	allow_take_control = true;
}

void tclacClimate::control(const ClimateCall &call) {
	// Não reenviar comandos que não alteram o estado conhecido.
	bool changed = false;
	if (call.get_mode().has_value() && call.get_mode().value() != this->mode)
		changed = true;
	if (call.get_target_temperature().has_value() &&
		(int) call.get_target_temperature().value() != (int) this->target_temperature)
		changed = true;
	if (call.get_fan_mode().has_value() &&
		(!this->fan_mode.has_value() || call.get_fan_mode().value() != this->fan_mode.value()))
		changed = true;
	if (call.get_swing_mode().has_value() && call.get_swing_mode().value() != this->swing_mode)
		changed = true;
	if (call.get_preset().has_value() &&
		(!this->preset.has_value() || call.get_preset().value() != this->preset.value()))
		changed = true;

	if (!changed) {
		ESP_LOGD("TCL", "Climate command has no changes, skipped");
		return;
	}

	if (call.get_mode().has_value()) {
		switch_climate_mode = call.get_mode().value();
	} else {
		switch_climate_mode = mode;
	}

	if (call.get_preset().has_value()) {
		switch_preset = call.get_preset().value();
	} else {
		switch_preset = preset.value_or(ClimatePreset::CLIMATE_PRESET_NONE);
	}

	if (call.get_fan_mode().has_value()) {
		switch_fan_mode = call.get_fan_mode().value();
	} else {
		switch_fan_mode = fan_mode.value_or(climate::CLIMATE_FAN_AUTO);
	}

	if (call.get_swing_mode().has_value()) {
		switch_swing_mode = call.get_swing_mode().value();
	} else {
		switch_swing_mode = swing_mode;
	}

	if (call.get_target_temperature().has_value()) {
		target_temperature_set = 31 - (int) call.get_target_temperature().value();
	} else {
		float temp = std::isnan(target_temperature) ? 18.0f : target_temperature;
		target_temperature_set = 31 - (int) temp;
	}

	is_call_control = true;
	takeControl();
	allow_take_control = true;
}
	
	
void tclacClimate::takeControl() {
	
	dataTX[7]  = 0b00000000;
	dataTX[8]  = 0b00000000;
	dataTX[9]  = 0b00000000;
	dataTX[10] = 0b00000000;
	dataTX[11] = 0b00000000;
	dataTX[19] = 0b00000000;
	dataTX[32] = 0b00000000;
	dataTX[33] = 0b00000000;
	
	if (is_call_control != true){
		switch_climate_mode = mode;
		switch_preset = preset.value_or(ClimatePreset::CLIMATE_PRESET_NONE);
		switch_fan_mode = fan_mode.value_or(climate::CLIMATE_FAN_AUTO);
		switch_swing_mode = swing_mode;
		float temp = std::isnan(target_temperature) ? 18.0f : target_temperature;
		target_temperature_set = 31-(int)temp;
	}
	
	if (beeper_status_) dataTX[7] += 0b00100000;
	
	if ((display_status_) && (switch_climate_mode != climate::CLIMATE_MODE_OFF)){
		dataTX[7] += 0b01000000;
	}
		
	switch (switch_climate_mode) {
		case climate::CLIMATE_MODE_OFF:
			dataTX[7] += 0b00000000;
			dataTX[8] += 0b00000000;
			break;
		case climate::CLIMATE_MODE_AUTO:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00001000;
			break;
		case climate::CLIMATE_MODE_COOL:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000011;	
			break;
		case climate::CLIMATE_MODE_DRY:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000010;	
			break;
		case climate::CLIMATE_MODE_FAN_ONLY:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000111;	
			break;
		case climate::CLIMATE_MODE_HEAT:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000001;	
			break;
	}

	switch(switch_fan_mode) {
		case climate::CLIMATE_FAN_AUTO:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000000;
			break;
		case climate::CLIMATE_FAN_QUIET:
			dataTX[8]	+= 0b10000000;
			dataTX[10]	+= 0b00000000;
			break;
		case climate::CLIMATE_FAN_LOW:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000001;
			break;
		case climate::CLIMATE_FAN_MIDDLE:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000110;
			break;
		case climate::CLIMATE_FAN_MEDIUM:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000011;
			break;
		case climate::CLIMATE_FAN_HIGH:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000111;
			break;
		case climate::CLIMATE_FAN_FOCUS:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000101;
			break;
		case climate::CLIMATE_FAN_DIFFUSE:
			dataTX[8]	+= 0b01000000;
			dataTX[10]	+= 0b00000000;
			break;
	}
	
	switch(switch_swing_mode) {
		case climate::CLIMATE_SWING_OFF:
			dataTX[10]	+= 0b00000000;
			dataTX[11]	+= 0b00000000;
			break;
		case climate::CLIMATE_SWING_VERTICAL:
			dataTX[10]	+= 0b00111000;
			dataTX[11]	+= 0b00000000;
			break;
		case climate::CLIMATE_SWING_HORIZONTAL:
			dataTX[10]	+= 0b00000000;
			dataTX[11]	+= 0b00001000;
			break;
		case climate::CLIMATE_SWING_BOTH:
			dataTX[10]	+= 0b00111000;
			dataTX[11]	+= 0b00001000;  
			break;
	}
	
	switch(switch_preset) {
		case ClimatePreset::CLIMATE_PRESET_NONE: break;
		case ClimatePreset::CLIMATE_PRESET_ECO: dataTX[7] += 0b10000000; break;
		case ClimatePreset::CLIMATE_PRESET_SLEEP: dataTX[19] += 0b00000001; break;
		case ClimatePreset::CLIMATE_PRESET_COMFORT: dataTX[8] += 0b00010000; break;
	}

	switch(vertical_swing_direction_) {
		case VerticalSwingDirection::UP_DOWN: dataTX[32] += 0b00001000; break;
		case VerticalSwingDirection::UPSIDE: dataTX[32] += 0b00010000; break;
		case VerticalSwingDirection::DOWNSIDE: dataTX[32] += 0b00011000; break;
	}
	switch(horizontal_swing_direction_) {
		case HorizontalSwingDirection::LEFT_RIGHT: dataTX[33] += 0b00001000; break;
		case HorizontalSwingDirection::LEFTSIDE: dataTX[33] += 0b00010000; break;
		case HorizontalSwingDirection::CENTER: dataTX[33] += 0b00011000; break;
		case HorizontalSwingDirection::RIGHTSIDE: dataTX[33] += 0b00100000; break;
	}
	switch(vertical_direction_) {
		case AirflowVerticalDirection::LAST: dataTX[32] += 0b00000000; break;
		case AirflowVerticalDirection::MAX_UP: dataTX[32] += 0b00000001; break;
		case AirflowVerticalDirection::UP: dataTX[32] += 0b00000010; break;
		case AirflowVerticalDirection::CENTER: dataTX[32] += 0b00000011; break;
		case AirflowVerticalDirection::DOWN: dataTX[32] += 0b00000100; break;
		case AirflowVerticalDirection::MAX_DOWN: dataTX[32] += 0b00000101; break;
	}
	switch(horizontal_direction_) {
		case AirflowHorizontalDirection::LAST: dataTX[33] += 0b00000000; break;
		case AirflowHorizontalDirection::MAX_LEFT: dataTX[33] += 0b00000001; break;
		case AirflowHorizontalDirection::LEFT: dataTX[33] += 0b00000010; break;
		case AirflowHorizontalDirection::CENTER: dataTX[33] += 0b00000011; break;
		case AirflowHorizontalDirection::RIGHT: dataTX[33] += 0b00000100; break;
		case AirflowHorizontalDirection::MAX_RIGHT: dataTX[33] += 0b00000101; break;
	}

	dataTX[9] = target_temperature_set;
		
	dataTX[0] = 0xBB;
	dataTX[1] = 0x00;
	dataTX[2] = 0x01;
	dataTX[3] = 0x03;
	dataTX[4] = 0x20;
	dataTX[5] = 0x03;
	dataTX[6] = 0x01;
	dataTX[12] = 0x00;
	dataTX[13] = 0x01;
	dataTX[14] = 0x00;
	dataTX[15] = 0x00;
	dataTX[16] = 0x00;
	dataTX[17] = 0x00;
	dataTX[18] = 0x00;
	dataTX[20] = 0x00;
	dataTX[21] = 0x00;
	dataTX[22] = 0x00;
	dataTX[23] = 0x00;
	dataTX[24] = 0x00;
	dataTX[25] = 0x00;
	dataTX[26] = 0x00;
	dataTX[27] = 0x00;
	dataTX[28] = 0x00;
	dataTX[30] = 0x00;
	dataTX[31] = 0x00;
	dataTX[34] = 0x00;
	dataTX[35] = 0x00;
	dataTX[36] = 0x00;
	dataTX[37] = 0xFF;
	dataTX[37] = tclacClimate::getChecksum(dataTX, sizeof(dataTX));

	tclacClimate::sendData(dataTX, sizeof(dataTX));
	allow_take_control = false;
	is_call_control = false;
}

void tclacClimate::sendData(byte * message, byte size) {
	tclacClimate::dataShow(1,1);
	this->tx_size_ = size;

	for (uint8_t k = 0; k < TX_REPEAT; k++) {
		if (k == 0) {
			this->try_send_frame_(0, TX_MAX_DEFERS);
		} else {
			this->set_timeout(k * TX_REPEAT_SPACING_MS, [this, k]() {
				this->try_send_frame_(k, TX_MAX_DEFERS);
			});
		}
	}

	ESP_LOGD("TCL", "Message queued after UART quiet check");
	tclacClimate::dataShow(1,0);
}

bool tclacClimate::bus_quiet_() {
	const uint32_t now = millis();

	if (esphome::uart::UARTDevice::available() > 0)
		return false;

	if (now - this->last_rx_ms_ < BUS_QUIET_MS)
		return false;

	// Depois de um poll, aguardar o início da resposta do AC antes de falar.
	if (now - this->poll_sent_ms_ < POLL_RESPONSE_WINDOW_MS &&
		(int32_t) (this->last_rx_ms_ - this->poll_sent_ms_) < 0)
		return false;

	return true;
}

void tclacClimate::try_send_frame_(uint8_t attempt, uint8_t defers_left) {
	if (!this->bus_quiet_() && defers_left > 0) {
		this->set_timeout(BUS_QUIET_MS, [this, attempt, defers_left]() {
			this->try_send_frame_(attempt, defers_left - 1);
		});
		return;
	}

	this->esphome::uart::UARTDevice::write_array(this->dataTX, this->tx_size_);
	this->esphome::uart::UARTDevice::flush();
}

String tclacClimate::getHex(byte *message, byte size) {
	String raw;
	for (int i = 0; i < size; i++) {
		raw += "\n" + String(message[i]);
	}
	raw.toUpperCase();
	return raw;
}

byte tclacClimate::getChecksum(const byte * message, size_t size) {
	byte position = size - 1;
	byte crc = 0;
	for (int i = 0; i < position; i++)
		crc ^= message[i];
	return crc;
}

void tclacClimate::dataShow(bool flow, bool shine) {
	if (module_display_status_){
		if (flow == 0){
			if (shine == 1){
#ifdef CONF_RX_LED
				this->rx_led_pin_->digital_write(true);
#endif
			} else {
#ifdef CONF_RX_LED
				this->rx_led_pin_->digital_write(false);
#endif
			}
		}
		if (flow == 1) {
			if (shine == 1){
#ifdef CONF_TX_LED
				this->tx_led_pin_->digital_write(true);
#endif
			} else {
#ifdef CONF_TX_LED
				this->tx_led_pin_->digital_write(false);
#endif
			}
		}
	}
}

void tclacClimate::set_beeper_state(bool state) {
	this->beeper_status_ = state;
	if (force_mode_status_ && allow_take_control) tclacClimate::takeControl();
}
void tclacClimate::set_display_state(bool state) {
	this->display_status_ = state;
	if (force_mode_status_ && allow_take_control) tclacClimate::takeControl();
}
void tclacClimate::set_force_mode_state(bool state) {
	this->force_mode_status_ = state;
}
#ifdef CONF_RX_LED
void tclacClimate::set_rx_led_pin(GPIOPin *rx_led_pin) {
	this->rx_led_pin_ = rx_led_pin;
}
#endif
#ifdef CONF_TX_LED
void tclacClimate::set_tx_led_pin(GPIOPin *tx_led_pin) {
	this->tx_led_pin_ = tx_led_pin;
}
#endif
void tclacClimate::set_module_display_state(bool state) {
	this->module_display_status_ = state;
}
void tclacClimate::set_vertical_airflow(AirflowVerticalDirection direction) {
	this->vertical_direction_ = direction;
	if (force_mode_status_ && allow_take_control) tclacClimate::takeControl();
}
void tclacClimate::set_horizontal_airflow(AirflowHorizontalDirection direction) {
	this->horizontal_direction_ = direction;
	if (force_mode_status_ && allow_take_control) tclacClimate::takeControl();
}
void tclacClimate::set_vertical_swing_direction(VerticalSwingDirection direction) {
	this->vertical_swing_direction_ = direction;
	if (force_mode_status_ && allow_take_control) tclacClimate::takeControl();
}
void tclacClimate::set_supported_modes(climate::ClimateModeMask modes) {
	this->supported_modes_ = modes;
}
void tclacClimate::set_horizontal_swing_direction(HorizontalSwingDirection direction) {
	horizontal_swing_direction_ = direction;
	if (force_mode_status_ && allow_take_control) tclacClimate::takeControl();
}
void tclacClimate::set_supported_fan_modes(climate::ClimateFanModeMask modes){
	this->supported_fan_modes_ = modes;
}
void tclacClimate::set_supported_swing_modes(climate::ClimateSwingModeMask modes) {
	this->supported_swing_modes_ = modes;
}
void tclacClimate::set_supported_presets(climate::ClimatePresetMask presets) {
  this->supported_presets_ = presets;
}

}
}
