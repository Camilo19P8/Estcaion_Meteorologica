#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

const char* ssid = "CLARO_ACF0";
const char* password = "194112045";

void setup() {
  Serial.begin(9600);

  // Conectar a la red Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Conectado a Wi-Fi");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    String serverPath = "http://192.168.0.12/ENMA_proyect/test_date.php";  // URL del servidor
    inputString = Serial.readString();  // Lee los datos recibidos y los convierte en String

    http.begin(client, serverPath + "?" + inputString);  // Cambiado para usar WiFiClient
    Serial.println(inputString);  // Imprime el string recibido

    int httpResponseCode = http.GET();  // Hace la solicitud GET

    if (httpResponseCode > 0) {
      String payload = http.getString();  // Obtiene la respuesta del servidor
      Serial.println(httpResponseCode);   // Muestra el código de respuesta HTTP
      Serial.println(payload);            // Muestra el cuerpo de la respuesta
    } else {
      Serial.print("Error en la solicitud HTTP: ");
      Serial.println(httpResponseCode);
    }

    http.end();  // Finaliza la conexión
  } else {
    Serial.println("Desconectado de Wi-Fi");
  }
  delay(5000);  // Espera 10 segundos antes de enviar otra solicitud
}

