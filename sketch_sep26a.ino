#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ======================================================
// WIFI
// ======================================================

const char* AP_SSID = "QUADRUPED-ESP32";
const char* AP_PASSWORD = "12345678";

WebServer server(80);


// ======================================================
// LEG SERVOS
// ======================================================

Servo leftup2;
Servo rightup2;
Servo leftdown2;
Servo rightdown2;


// Current leg position
int legPosition = 0;


// ======================================================
// FRONTEND
// ======================================================

const char MAIN_PAGE[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta name="viewport"
      content="width=device-width, initial-scale=1.0">

<title>Quadruped Robot</title>

<style>

* {
    box-sizing: border-box;
}

body {
    margin: 0;
    font-family: Arial, Helvetica, sans-serif;
    background: #111827;
    color: white;
    text-align: center;
}

.container {
    max-width: 500px;
    margin: auto;
    padding: 25px 20px;
}

h1 {
    font-size: 28px;
    margin-bottom: 5px;
}

.subtitle {
    color: #9ca3af;
    margin-bottom: 25px;
}

.robot-card {
    background: #1f2937;
    border-radius: 20px;
    padding: 25px;
    box-shadow: 0 10px 30px rgba(0,0,0,0.3);
}

.status {
    font-size: 22px;
    font-weight: bold;
    margin: 15px 0 25px;
}

.status-value {
    padding: 12px;
    border-radius: 12px;
    background: #374151;
}

button {
    width: 100%;
    padding: 20px;
    margin: 10px 0;
    border: none;
    border-radius: 15px;
    font-size: 21px;
    font-weight: bold;
    color: white;
    cursor: pointer;
    transition: 0.15s;
}

button:active {
    transform: scale(0.97);
}

.stand {
    background: #16a34a;
}

.sit {
    background: #dc2626;
}

.stop {
    background: #4b5563;
}

.info {
    margin-top: 25px;
    background: #111827;
    padding: 15px;
    border-radius: 12px;
    color: #9ca3af;
    font-size: 14px;
}

.footer {
    margin-top: 20px;
    font-size: 13px;
    color: #6b7280;
}

</style>

</head>


<body>

<div class="container">

    <h1>🤖 Quadruped Robot</h1>

    <div class="subtitle">
        ESP32 Robot Controller
    </div>


    <div class="robot-card">

        <div class="status">
            Current State
        </div>

        <div id="status"
             class="status-value">
            SITTING
        </div>


        <button class="stand"
                onclick="standRobot()">

            STAND UP

        </button>


        <button class="sit"
                onclick="sitRobot()">

            SIT DOWN

        </button>


        <button class="stop"
                onclick="getStatus()">

            REFRESH STATUS

        </button>


        <div class="info">

            ESP32 IP:
            <strong>192.168.4.1</strong>

            <br><br>

            Connect your phone to:
            <strong>QUADRUPED-ESP32</strong>

        </div>

    </div>


    <div class="footer">
        Quadruped Robot • ESP32
    </div>

</div>


<script>

function standRobot() {

    document.getElementById("status").innerText =
        "STANDING...";

    fetch("/stand")
    .then(response => response.text())
    .then(data => {

        document.getElementById("status").innerText =
            "STANDING";

    })
    .catch(error => {

        document.getElementById("status").innerText =
            "CONNECTION ERROR";

    });

}


function sitRobot() {

    document.getElementById("status").innerText =
        "SITTING...";

    fetch("/sit")
    .then(response => response.text())
    .then(data => {

        document.getElementById("status").innerText =
            "SITTING";

    })
    .catch(error => {

        document.getElementById("status").innerText =
            "CONNECTION ERROR";

    });

}


function getStatus() {

    fetch("/status")
    .then(response => response.text())
    .then(data => {

        document.getElementById("status").innerText =
            data;

    });

}

</script>

</body>

</html>

)rawliteral";


// ======================================================
// MOVE LEGS
// ======================================================

void moveLegs(int targetPosition) {

    Serial.print("Moving legs from ");
    Serial.print(legPosition);
    Serial.print(" to ");
    Serial.println(targetPosition);


    // Move upward
    if (legPosition < targetPosition) {

        for (int pos = legPosition;
             pos <= targetPosition;
             pos++) {

            leftup2.write(pos);
            rightup2.write(pos);
            leftdown2.write(pos);
            rightdown2.write(pos);

            delay(15);
        }
    }


    // Move downward
    else if (legPosition > targetPosition) {

        for (int pos = legPosition;
             pos >= targetPosition;
             pos--) {

            leftup2.write(pos);
            rightup2.write(pos);
            leftdown2.write(pos);
            rightdown2.write(pos);

            delay(15);
        }
    }


    legPosition = targetPosition;
}


// ======================================================
// HOME PAGE
// ======================================================

void handleRoot() {

    server.send(
        200,
        "text/html",
        MAIN_PAGE
    );

}


// ======================================================
// STAND
// ======================================================

void handleStand() {

    Serial.println("WEB COMMAND: STAND");

    moveLegs(90);

    server.send(
        200,
        "text/plain",
        "STANDING"
    );
}


// ======================================================
// SIT
// ======================================================

void handleSit() {

    Serial.println("WEB COMMAND: SIT");

    moveLegs(0);

    server.send(
        200,
        "text/plain",
        "SITTING"
    );
}


// ======================================================
// STATUS
// ======================================================

void handleStatus() {

    if (legPosition == 90) {

        server.send(
            200,
            "text/plain",
            "STANDING"
        );

    } else {

        server.send(
            200,
            "text/plain",
            "SITTING"
        );
    }
}


// ======================================================
// SETUP
// ======================================================

void setup() {

    Serial.begin(115200);


    // --------------------------------------------------
    // SERVO ATTACH
    // --------------------------------------------------

    leftup2.attach(16);
    rightup2.attach(26);
    leftdown2.attach(4);
    rightdown2.attach(2);

    Serial.println("4 leg servos attached");


    // --------------------------------------------------
    // INITIAL POSITION = SITTING
    // --------------------------------------------------

    leftup2.write(0);
    rightup2.write(0);
    leftdown2.write(0);
    rightdown2.write(0);

    legPosition = 0;


    delay(1000);


    // --------------------------------------------------
    // CREATE WIFI ACCESS POINT
    // --------------------------------------------------

    WiFi.softAP(AP_SSID, AP_PASSWORD);


    Serial.println();
    Serial.println("==============================");
    Serial.println("QUADRUPED ESP32");
    Serial.println("==============================");

    Serial.print("WiFi Name: ");
    Serial.println(AP_SSID);

    Serial.print("WiFi IP: ");
    Serial.println(WiFi.softAPIP());


    // --------------------------------------------------
    // WEB SERVER ROUTES
    // --------------------------------------------------

    server.on("/", handleRoot);

    server.on("/stand",
              HTTP_GET,
              handleStand);

    server.on("/sit",
              HTTP_GET,
              handleSit);

    server.on("/status",
              HTTP_GET,
              handleStatus);


    server.begin();

    Serial.println("Web server started");

    Serial.println("==============================");
}


// ======================================================
// LOOP
// ======================================================

void loop() {

    server.handleClient();

}