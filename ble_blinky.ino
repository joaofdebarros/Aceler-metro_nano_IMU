/*
   BLE IMU receiver (Etapa 1)

   Baseado no exemplo "BLE blinky" da Silicon Labs.

   O Arduino Nano Matter roda como BLE peripheral (GATT server). Ele expoe uma
   caracteristica gravavel "IMU" que recebe 3 floats (x, y, z) em little-endian
   (12 bytes no total). Um app no celular (via Web Bluetooth / WebBLE) le o
   acelerometro do proprio celular e ESCREVE esses valores nesta caracteristica.
   O firmware decodifica e imprime x/y/z na Serial (115200).

   Topologia:
     Celular (central, le o sensor)  --BLE write-->  Nano (peripheral)  --Serial-->  PC

   UUIDs (customizados para este projeto):
     Service      : 6d570001-9c8f-4f2b-9b6a-2a1b0c0d0e0f
     IMU (Write)  : 6d570002-9c8f-4f2b-9b6a-2a1b0c0d0e0f   (12 bytes: float x,y,z LE)

   Compativel com: Arduino Nano Matter e demais placas Silabs do exemplo original.
   Requer o protocol stack 'BLE (Silabs)' em 'Tools > Protocol stack'.

   Base original por: Tamas Jozsi (Silicon Labs)
 */

#include <string.h>

// Numero de valores float recebidos por escrita (x, y, z)
// A caracteristica IMU aceita dois formatos:
//   - 12 bytes: 3 floats  -> accel x,y,z            (compatibilidade)
//   - 24 bytes: 6 floats  -> accel x,y,z + orientacao beta,gamma,alpha
#define IMU_ACCEL_ONLY_BYTES  (3 * sizeof(float))   // 12 bytes
#define IMU_FULL_BYTES        (6 * sizeof(float))   // 24 bytes
#define IMU_MAX_BYTES         IMU_FULL_BYTES        // tamanho maximo da caracteristica

static void set_led_on(bool state);
static void ble_initialize_gatt_db();
static void ble_start_advertising();

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
  set_led_on(false);
  Serial.begin(115200);
  Serial.println("Silicon Labs BLE IMU receiver - Etapa 1");
}

void loop()
{
  // Todo o trabalho acontece no handler de eventos BLE (sl_bt_on_event).
}

static const uint8_t advertised_name[] = "Nano IMU";
static uint16_t gattdb_session_id;
static uint16_t generic_access_service_handle;
static uint16_t name_characteristic_handle;
static uint16_t imu_service_handle;
static uint16_t imu_data_characteristic_handle;

/**************************************************************************//**
 * Decodifica os floats recebidos (little-endian) e imprime na Serial.
 *   12 bytes -> accel x,y,z
 *   24 bytes -> accel x,y,z + orientacao beta,gamma,alpha (graus)
 *****************************************************************************/
static void handle_imu_write(const uint8_t* data, size_t len)
{
  if (len != IMU_ACCEL_ONLY_BYTES && len != IMU_FULL_BYTES) {
    Serial.print("IMU: tamanho inesperado (");
    Serial.print(len);
    Serial.println(" bytes; esperado 12 ou 24)");
    return;
  }

  // EFR32 (ARM Cortex-M) e o browser usam little-endian, entao o layout de
  // bytes bate com o float nativo (copia direta).
  float v[6] = { 0 };
  memcpy(v, data, len);

  Serial.print("ACC  x=");
  Serial.print(v[0], 3);
  Serial.print("  y=");
  Serial.print(v[1], 3);
  Serial.print("  z=");
  Serial.print(v[2], 3);

  if (len == IMU_FULL_BYTES) {
    // beta  = inclinacao frente/tras (-180..180)
    // gamma = inclinacao esquerda/direita (-90..90)
    // alpha = bussola / rotacao no plano (0..360)
    Serial.print("   |  ORI  beta=");
    Serial.print(v[3], 2);
    Serial.print("  gamma=");
    Serial.print(v[4], 2);
    Serial.print("  alpha=");
    Serial.print(v[5], 2);
  }
  Serial.println();

  // Feedback visual: pisca o LED a cada pacote recebido.
  static bool led = false;
  led = !led;
  set_led_on(led);
}

/**************************************************************************//**
 * Bluetooth stack event handler
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  switch (SL_BT_MSG_ID(evt->header)) {
    // O radio esta pronto. Nao chame comandos da stack antes deste evento.
    case sl_bt_evt_system_boot_id:
    {
      Serial.begin(115200);
      Serial.println("BLE stack booted");
      ble_initialize_gatt_db();
      ble_start_advertising();
      Serial.println("BLE advertisement started (nome: Nano IMU)");
    }
    break;

    case sl_bt_evt_connection_opened_id:
      Serial.println("BLE connection opened");
      break;

    case sl_bt_evt_connection_closed_id:
      Serial.println("BLE connection closed");
      set_led_on(false);
      ble_start_advertising();
      Serial.println("BLE advertisement restarted");
      break;

    // Um cliente remoto escreveu num atributo do nosso GATT local.
    case sl_bt_evt_gatt_server_attribute_value_id:
      if (evt->data.evt_gatt_server_attribute_value.attribute == imu_data_characteristic_handle) {
        handle_imu_write(evt->data.evt_gatt_server_attribute_value.value.data,
                         evt->data.evt_gatt_server_attribute_value.value.len);
      }
      break;

    default:
      break;
  }
}

/**************************************************************************//**
 * Liga/desliga o LED embutido, respeitando a logica invertida de algumas placas.
 *****************************************************************************/
static void set_led_on(bool state)
{
  if (state) {
    digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE);
  } else {
    digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
  }
}

/**************************************************************************//**
 * Inicia o advertisement BLE (inicializa na primeira chamada).
 *****************************************************************************/
static void ble_start_advertising()
{
  static uint8_t advertising_set_handle = 0xff;
  static bool init = true;
  sl_status_t sc;

  if (init) {
    sc = sl_bt_advertiser_create_set(&advertising_set_handle);
    app_assert_status(sc);

    // Intervalo de advertising ~100ms (valor * 0.625ms)
    sc = sl_bt_advertiser_set_timing(advertising_set_handle, 160, 160, 0, 0);
    app_assert_status(sc);

    init = false;
  }

  sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle, sl_bt_advertiser_general_discoverable);
  app_assert_status(sc);

  sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
  app_assert_status(sc);
}

/**************************************************************************//**
 * Inicializa a base de dados GATT: Generic Access + servico IMU customizado.
 *****************************************************************************/
static void ble_initialize_gatt_db()
{
  sl_status_t sc;
  sc = sl_bt_gattdb_new_session(&gattdb_session_id);
  app_assert_status(sc);

  // --- Generic Access service (0x1800) com Device Name (0x2A00) ---
  const uint8_t generic_access_service_uuid[] = { 0x00, 0x18 };
  sc = sl_bt_gattdb_add_service(gattdb_session_id,
                                sl_bt_gattdb_primary_service,
                                SL_BT_GATTDB_ADVERTISED_SERVICE,
                                sizeof(generic_access_service_uuid),
                                generic_access_service_uuid,
                                &generic_access_service_handle);
  app_assert_status(sc);

  const sl_bt_uuid_16_t device_name_characteristic_uuid = { .data = { 0x00, 0x2A } };
  sc = sl_bt_gattdb_add_uuid16_characteristic(gattdb_session_id,
                                              generic_access_service_handle,
                                              SL_BT_GATTDB_CHARACTERISTIC_READ,
                                              0x00,
                                              0x00,
                                              device_name_characteristic_uuid,
                                              sl_bt_gattdb_fixed_length_value,
                                              sizeof(advertised_name) - 1,
                                              sizeof(advertised_name) - 1,
                                              advertised_name,
                                              &name_characteristic_handle);
  app_assert_status(sc);

  sc = sl_bt_gattdb_start_service(gattdb_session_id, generic_access_service_handle);
  app_assert_status(sc);

  // --- Servico IMU customizado ---
  // UUID: 6d570001-9c8f-4f2b-9b6a-2a1b0c0d0e0f  (bytes em ordem reversa/LE)
  const uuid_128 imu_service_uuid = {
    .data = { 0x0f, 0x0e, 0x0d, 0x0c, 0x1b, 0x2a, 0x6a, 0x9b, 0x2b, 0x4f, 0x8f, 0x9c, 0x01, 0x00, 0x57, 0x6d }
  };
  sc = sl_bt_gattdb_add_service(gattdb_session_id,
                                sl_bt_gattdb_primary_service,
                                SL_BT_GATTDB_ADVERTISED_SERVICE,
                                sizeof(imu_service_uuid),
                                imu_service_uuid.data,
                                &imu_service_handle);
  app_assert_status(sc);

  // Caracteristica IMU (Write / Write Without Response): tamanho variavel.
  // 12 bytes (accel x,y,z) ou 24 bytes (accel + orientacao), floats LE.
  // UUID: 6d570002-9c8f-4f2b-9b6a-2a1b0c0d0e0f
  const uuid_128 imu_data_characteristic_uuid = {
    .data = { 0x0f, 0x0e, 0x0d, 0x0c, 0x1b, 0x2a, 0x6a, 0x9b, 0x2b, 0x4f, 0x8f, 0x9c, 0x02, 0x00, 0x57, 0x6d }
  };
  uint8_t imu_init_value[IMU_MAX_BYTES] = { 0 };
  sc = sl_bt_gattdb_add_uuid128_characteristic(gattdb_session_id,
                                               imu_service_handle,
                                               SL_BT_GATTDB_CHARACTERISTIC_WRITE | SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE,
                                               0x00,
                                               0x00,
                                               imu_data_characteristic_uuid,
                                               sl_bt_gattdb_variable_length_value,
                                               IMU_MAX_BYTES,              // max length
                                               IMU_MAX_BYTES,              // initial value length
                                               imu_init_value,             // initial value
                                               &imu_data_characteristic_handle);
  app_assert_status(sc);

  sc = sl_bt_gattdb_start_service(gattdb_session_id, imu_service_handle);
  app_assert_status(sc);

  sc = sl_bt_gattdb_commit(gattdb_session_id);
  app_assert_status(sc);
}

#ifndef ARDUINO_SILABS_STACK_BLE_SILABS
  #error "This example is only compatible with the Silicon Labs BLE stack. Please select 'BLE (Silabs)' in 'Tools > Protocol stack'."
#endif
