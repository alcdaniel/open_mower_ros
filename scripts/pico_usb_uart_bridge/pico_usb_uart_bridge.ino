// Raspberry Pi Pico USB <-> UART bridge for Raspberry Pi 4 U-Boot/Linux serial console.
// Based on OpenMower SerialRedirect concept, without power-control pin logic.

static constexpr uint32_t UART_BAUD = 115200;
static constexpr int UART_TX_PIN = 0;  // GP0 -> Pi4 RXD (GPIO15, physical pin 10)
static constexpr int UART_RX_PIN = 1;  // GP1 <- Pi4 TXD (GPIO14, physical pin 8)

void setup() {
  Serial.begin(115200);        // USB CDC (baud ignored on USB)
  Serial1.setTX(UART_TX_PIN);  // UART0 TX
  Serial1.setRX(UART_RX_PIN);  // UART0 RX
  Serial1.begin(UART_BAUD);

  // Allow hosts that don't assert DTR to still send data.
  Serial.ignoreFlowControl(true);

  // Don't block forever if no terminal is attached yet.
  unsigned long start = millis();
  while (!Serial && (millis() - start < 1500)) {
    delay(10);
  }

  delay(200);
  while (Serial.available()) {
    Serial.read();
  }

  Serial.println("Pico USB-UART bridge ready (GP0 TX, GP1 RX, 115200)");
}

void loop() {
  while (Serial.available()) {
    Serial1.write(Serial.read());
  }

  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
