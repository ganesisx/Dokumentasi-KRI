# How To Use

- Include zip file yang terdapat dalam folder library
- Jalankan .ino dari file dalam folder get_bluetooth_MAC_Address untuk mengetahui bluetooth MACAddress dari ESP32 yang digunakan
- Connect PS4 ke device, run SixAxisPairTool. Masukin Bluetooth MACAddress yang sudah didapat sebelumnya ke dalam box yang tersedia
- Upload code dalam folder receive_data pada ESP32. 
- Connect PS4 ke ESP32 dengan menekan tmbol PSButton pada controller PS4. Sebelum connect, lampu LED blinking, setelah connect LED solid
- Lihat output pada serial monitor apakah sudah seuai untuk semua button.
- Upload code dalam folder joystick_PS4_ESP32 untuk memberikan input ke Nucleo

<br>

` **IMPORTANT** ` 

```
Proses ini dilakukan untuk versi Arduino IDE ESP 32 version 2.0.7
```