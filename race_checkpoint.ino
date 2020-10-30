/* Output-side (LED) Arduino code */ #include "SoftwareSerial.h"
 // RX: Arduino pin 2, XBee pin DOUT.  TX:  Arduino pin 3, XBee pin DIN
SoftwareSerial XBee(1, 0);
int LED = 9;

// 거리 센서
int echoPin = 6;
int trigPin = 7;

void setup() {
    // Baud rate MUST match XBee settings (as set in XCTU)
    XBee.begin(9600);
    pinMode(LED, OUTPUT);
    // trig를 출력모드로 설정, echo를 입력모드로 설정
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

int mode = 0; // 0: initial 1:receive range 2:race ready 3:sensor checked
int temp_1 = 0;
int temp_10 = 0;
int temp_100 = 0;
int dist = 120;

int get_dist(int max_distance) {
    // 초음파를 보낸다. 다 보내면 echo가 HIGH 상태로 대기하게 된다.
    digitalWrite(trigPin, HIGH);
    delay(10);
    digitalWrite(trigPin, LOW);

    // echoPin 이 HIGH를 유지한 시간을 저장 한다.
    double duration = pulseIn(echoPin, HIGH);
    // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
    return (int)(((double)(340 * duration) / 10000) / 2);
}

void loop() {
    if (mode != 2 && XBee.available()) {
        char c = XBee.read();
        Serial.println(c);

        // 1의자리수 세팅
        if (c >= 0 && c <= 9) {
            temp_1 = (int) c;
        }
        // 10의자리수 세팅
        if (c >= 10 && c <= 19) {
            temp_10 = ((int) c - 10) * 10;
        }
        // 100의자리수 세팅
        if (c >= 20 && c <= 29) {
            temp_100 = ((int) c - 20) * 100;
        }

        if (temp_1 >= 0 && temp_10 >= 0 && temp_100 >= 0) {
            // 숫자 세팅 완료
            if (mode == 1) {
                dist = temp_1 + temp_10 + temp_100;
            }
        }

        // 초기 세팅 확인 모드0
        if (c == 31) {
            digitalWrite(LED, HIGH);
            delay(100);
            digitalWrite(LED, LOW);
            // initial mode mode=0
            // 세팅값 초기화
            temp_1 = -1;
            temp_10 = -1;
            temp_100 = -1;
            dist = 0;

            XBee.write((char) 32);
            XBee.flush();
            mode = 0;
        }

        // 거리 세팅 시작 모드1
        if (c == 41) {
            // range setting mode=1
            XBee.write((char) 42);
            XBee.flush();
            temp_1 = -1;
            temp_10 = -1;
            temp_100 = -1;
            mode = 1;
        }

        // 거리 세팅 완료 확인
        if (c == 43) {
            if (dist != 0) {
                XBee.write((char) 44);
            } else {
                XBee.write((char) 45);
            }
            XBee.flush();
        }

        if (c == 51) {
            XBee.write((char) 52);
            XBee.flush();
            mode = 2;
        }

        if (c == 54) {
            if (mode == 3) {
                XBee.write((char) 53);
                XBee.flush();
            } else {
                XBee.write((char) 55);
                XBee.flush();
            }
            delay(200);
            return;
        }
    }

    if (mode == 2) {
        // 센서 체크
        digitalWrite(LED, HIGH);
        //    XBee.write(get_dist());
        //    delay(100);

        if (get_dist(dist) <= dist) {
            mode = 3;
        }
    } else {
        digitalWrite(LED, LOW);
    }

    if (mode == 3) {
        // 닿았음
    }

}
