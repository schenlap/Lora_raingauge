# Lora_raingauge
Raingaug based on CubeCell with LoraWan communication

Sketch runs on CubeCell HTCC-AB01

## Compile ##
Use adruino ide with cibecell lib https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/quick_start.html
You need at lease a version containing watchdog https://github.com/HelTecAutomation/ASR650x-Arduino/commit/a69825889bf3e34463dc76b31b435b764d647988

## Decode ##
Use can use this decoder in thethingsnetwork to decode messages
```
function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};

  var bat_mV;
  var rain_total_cnt;
  var rain_total;
  var restart;

  if (port == 1 || port == 2) {
    bat_mV = (bytes[0] | bytes[1] << 8) / 1000;
    rain_total_cnt = (bytes[2] | bytes[3] << 8);
    rain_total = rain_total_cnt; // * 1.17 /* l/m^2: 9.85cm Durchmesser, 28x / 250ml */
    restart = bytes[4];
  }
  return {
    field1: bat_mV,
    field2: rain_total,
    field3: restart,
  };
}
```
