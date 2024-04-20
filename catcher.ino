#include <ArduinoJson.h>

void setup(){
    Serial.begin(9600);
}

void loop(){
    if(Serial.available() > 0){
        String data = Serial.readString();

        DynamicJsonDocument doc(1024);

        DeserializationError error = deserializeJson(doc, data);

        if(error){
            Serial.print("parsing error");
            Serial.println(error.c_str());
            return;
        }

        for(size_t i = 0; i < doc.size(); i++){
            JsonObject detection = doc[i];
            
            
            // Extract information
            String nama_kelas = detection["kelas"].as<String>();
            float keyakinan = detection["keyakinan"].as<float>();

            // For array elements, use get or as method
            int koordinat_x = detection["koordinat"][0].as<int>();
            int koordinat_y = detection["koordinat"][1].as<int>();

            // Print extracted information
            Serial.print("Nama Kelas: ");
            Serial.println(nama_kelas);
            Serial.print("Keyakinan: ");
            Serial.println(keyakinan);
            Serial.print("Koordinat X: ");
            Serial.println(koordinat_x);
            Serial.print("Koordinat Y: ");
            Serial.println(koordinat_y);
 

        }
      
    }
}