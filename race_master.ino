#include <Wire.h>

#include <LiquidCrystal_I2C.h>

#include "SoftwareSerial.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial XBee(1, 0);

int echoPin = 12; // 초음파 센서 에코핀
int trigPin = 13; // 초음파 센서 트리거핀
int led_pin_red = 4;
int led_pin_green = 5;
int led_pin_blue = 6;

int mode = 0; // 0: 대기, 1: stable(stopwatch_wait), 2: stopwatch_start, 3: stopwatch_finish
int saved_dist; // 거리 변경을 체크하기 위한 변수. cm단위(int)
double last_dist_change; // 거리 변경되면 업데이트되는 변수. millis() / 1000 가 주로 저장됨.
double stopwatch_start; // 스탑워치의 시작시간 millis() / 1000 가 주로 저장됨.
double stopwatch_end; // 스탑워치의 끝나는 시각 millis() / 1000 가 주로 저장됨.
double stopwatch_advantage; // 초음파 센서 pulseIn 이 너무 느려 거리에 비례해 오프셋 조정하려 했는데 실패함. 버려진 변수. 코드에는 사용됨.

bool sub_available_check = false; // 서브 모듈이 사용 가능한지 확인 여부
bool sub_available = false; // 서브 모듈이 사용 가능한지 여부 
bool sub_range_ready = false; // 서브 모듈에 거리가 세팅 완료 되었는지 여부
bool sub_ready_sent = false; // 서브 모듈에게 시작 전 준비 상태인지 확인 메시지를 보냈는지 여부
bool sub_ready = false; // 서브 모듈이 시작 전 준비 상태인지 확인 완료 여부
bool sub_accepted = false; // 사용자가 서브 모듈에 닿았는지 여부


int DIST_MAX = 1190; // 센서가 거리를 잴 수 있는 최대 범위(cm)
int stable_wait_delay = 5; // 안정화까지의 대기 시간(초)

// ------- 서브 모듈과의 통신 코드 안내 ------- //
// 31 - 서브접속체크
// 32 - (응답)접속완료
// 41 - 거리셋팅모드진입
// 42 - (응답)거리셋팅모드진입확인
// 43 - 거리셋팅완료확인
// 44 - (응답)거리셋팅완료
// 45 - (응답)거리셋팅미완료
// 51 - 출발전준비상태확인
// 52 - (응답)출발전준비완료
// 53 - (응답)센서에닿았음
// 54 - 센서에닿은지확인
// 55 - (응답)센서에닿지않았음
// ----------------------------------------- //

void setup() {
    // XBee 통신을 시작함
    XBee.begin(9600);

    // lcd 모듈을 시작하고 백라이트를 켬
    lcd.begin();
    lcd.backlight();

    // 초음파 센서의 trig를 출력모드로 설정, echo를 입력모드로 설정
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // 3색 led 설정
    pinMode(led_pin_red, OUTPUT);
    pinMode(led_pin_green, OUTPUT);
    pinMode(led_pin_blue, OUTPUT);
}

// XBee 수신 버퍼를 비움
void empty_buffer() {
    while (XBee.available()) {
        XBee.read();
    }
}

// lcd의 1번째줄에 거리를 표시함
void display_dist(int dist) {
    lcd.setCursor(0, 0);
    if (dist >= DIST_MAX) {
        lcd.print("|---TOO FAR!--->");
    } else {
        lcd.print("|<----");

        if (dist / 100 < 1) {
            lcd.print("-");
            if (dist / 10 < 1)
                lcd.print("-");
        }

        lcd.print(dist);
        lcd.print("----->|");
    }
}

// lcd를 빈 화면으로 채움
void lcd_clear(int mode) {
    if (mode == 0 || mode == 1) {
        lcd.setCursor(0, 0);
        lcd.print("                ");
    }

    if (mode == 0 || mode == 2) {
        lcd.setCursor(0, 1);
        lcd.print("                ");
    }
    lcd.setCursor(0, 0);
}

// led의 상태를 제어함.
void led_control(bool r, bool g, bool b) {
    digitalWrite(led_pin_red, r ? HIGH : LOW);
    digitalWrite(led_pin_green, g ? HIGH : LOW);
    digitalWrite(led_pin_blue, b ? HIGH : LOW);
}

// 서브 센서에 닿았는지 확인하는 함수
bool read_accept_from_sub() {
    // 버퍼를 비움
    empty_buffer();

    // 54(센서에닿은지확인)
    XBee.write((char) 54);
    XBee.flush();
    delay(50);

    int tries = 0;
    while (tries < 500) {
        // 500회 동안 버퍼를 읽고 원하는 값이 나오지 않으면 종료함.
        tries++;

        if (XBee.available()) {
            char c = XBee.read();
            
            if (c == 53) {
                // 53(센서에닿았음)
                return true;
            } else if (c == 55) {
                // 55(센서에닿지않았음)
                return false;
            }
        }
    }

    // 500회 동안 버퍼를 읽고 원하는 값이 나오지 않으면 안 닿았다고 간주함.
    return false;
}

// 준비 상태 서브로 전송하는 함수
void set_ready_to_sub() {
    lcd_clear(0);

    bool setOK = false;

    lcd.setCursor(0, 0);

    for (int tries = 0; tries < 5; tries++) {
        lcd.setCursor(0, 0);
        lcd.print("SEND READY SIGN ");
        lcd.setCursor(0, 1);
        lcd.print(tries);
        lcd.print("/5");

        // 버퍼 클리어
        empty_buffer();

        // 준비 신호 전송
        XBee.write((char) 51);
        XBee.flush();
        delay(50);

        // 수신할때까지 대기
        int recv_try = 0;
        while (setOK == false && recv_try < 500) {
            recv_try++;

            if (XBee.available()) {
                char c = XBee.read();
                // 응답 오면 리턴
                if (c == 52) {
                    setOK = true;
                }
            }

            delay(100);
        }

        if (setOK == true)
            break;
    }

    return;
}

// 서브에 거리 설정
void set_distance_to_sub() {
    lcd_clear(0);

    bool setOK = false;

    lcd.setCursor(0, 0);
    lcd.print("CONNECT TO SUB..");

    // 버퍼 클리어
    empty_buffer();

    // 41: 거리세팅모드진입
    XBee.write((char) 41);
    XBee.flush();

    // 먼저 거리세팅 모드 진입
    while (setOK == false) {
        if (XBee.available()) {
            char c = XBee.read();
            // 거리셋팅모드확인 응답 오면 다음단계
            if (c == 42) {
                setOK = true;
            }
        }
        delay(100);
    }

    lcd.setCursor(0, 0);
    lcd.print("SETTING RANGE...    ");

    // 버퍼 클리어
    empty_buffer();

    // 실제 거리세팅 진행
    // 60 cm
    XBee.write((char) 0);
    XBee.write((char) 16);
    XBee.write((char) 20);
    XBee.flush();
    delay(100);

    // 저장되었는지 확인
    XBee.write((char) 43);
    XBee.flush();
    delay(100);

    setOK = false;
    while (setOK == false) {
        if (XBee.available()) {
            char c = XBee.read();
            // 거리세팅완료확인
            if (c == 44) {
                setOK = true;
            } else if (c == 45) {
                // 거리세팅미완료응답시 한번더 전송
                // 버퍼 클리어
                empty_buffer();
                XBee.write((char) 0);
                XBee.write((char) 16);
                XBee.write((char) 21);
                XBee.flush();
                delay(500);
            }
        }

        delay(100);
    }

    delay(500);
    // 버퍼 클리어
    empty_buffer();
}

// 서브가 접속되어있는지 확인
bool connect_to_sub() {
    bool connectOK = false;
    int loop_cnt = 0;

    lcd.setCursor(0, 0);
    lcd.print("CONNECT TO SUB  ");

    // 버퍼 클리어
    empty_buffer();
    XBee.write((char) 31);
    XBee.flush();

    while (connectOK == false && loop_cnt < 100) {
        lcd.setCursor(0, 1);
        lcd.print("TRY: ");
        lcd.print(loop_cnt);
        lcd.print("/100");

        loop_cnt++;

        if (XBee.available()) {
            char c = XBee.read();
            // 서브 응답 오는지 확인
            if (c == 32) {
                connectOK = true;
            } else if (loop_cnt == 50) {
                // 루프 카운트가 50회 될때까지 응답이 안 오면 다시 보냄
                XBee.write((char) 31);
                XBee.flush();
            }
        }

        delay(100);
    }

    empty_buffer();
    return connectOK;
}

int get_distance() {
    double duration, distance;

    // 초음파를 보낸다. 다 보내면 echo가 HIGH 상태로 대기하게 된다.
    digitalWrite(trigPin, HIGH);
    delay(10);
    digitalWrite(trigPin, LOW);

    // echoPin 이 HIGH를 유지한 시간을 저장 한다.
    duration = pulseIn(echoPin, HIGH);

    // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
    distance = (int)(((double)(340 * duration) / 10000) / 2);

    return distance;
}

// 현재 시각 반환
double current_time() {
    return (double) millis() / (double) 1000;
}

// 거리가 바뀌었는지 확인 +5cm -5cm 
bool is_distance_changed(int distance) {
    return (saved_dist - 5 > (int) distance || saved_dist + 5 < (int) distance);
}

// 저장된 거리값을 변경함
void change_saved_distance(int distance) {
    saved_dist = distance;
}

// last_dist_change 를 현재 시각으로 변경함
void touch_last_dist_change() {
    last_dist_change = current_time();
}

// last_dist_change 로부터 n 초동안 흘렀는지 확인
bool is_delayed_for(double seconds) {
    return last_dist_change + seconds < current_time();
}

// 모드 0, 거리 안정화 단계
void mode_0() {
    // sub 사용 가능한지 체크, 사용 가능하면 sub_available_check 변경.
    // 한 번만 체크
    if (!sub_available_check) {
        sub_available = connect_to_sub();
        sub_available_check = true;
    }

    if (sub_available && !sub_range_ready) {
        set_distance_to_sub();
        sub_range_ready = true;
    }

    // 초음파 센서로부터 거리 가져옴
    double distance = get_distance();
    // lcd에 거리 표기
    lcd.setCursor(0, 0);
    display_dist(distance);

    // 거리 달라졌는지 확인
    if (is_distance_changed(distance)) {
        // 거리 달라지면 값 변경
        touch_last_dist_change();
        change_saved_distance((int) distance);
    } else {
        // 거리 달라지지 않은 경우
        if (distance < 10 || distance > DIST_MAX) {
            // 거리가 10cm 보다 까까움. 또는 너무 멈.
            millis() / 50 % 2 ? led_control(true, false, false) : led_control(false, false, false);
            lcd.setCursor(0, 1);
            lcd.print("CHECK DISTANCE ");
        } else if (is_delayed_for(stable_wait_delay)) {
            // 안정화 된 경우 다음 단계로
            touch_last_dist_change();
            change_saved_distance((int) distance);

            lcd_clear(0);
            mode = 1;
        } else {
            // 불안정 상태인 경우 남은 시간 표시
            lcd.setCursor(0, 1);
            lcd.print("WAITING... ");
            led_control(false, false, false);
            lcd.print((double) stable_wait_delay - (current_time() - last_dist_change));
            lcd.print("      ");
        }
    }
}

// 안정화 되었고 시작 대기 모드
void mode_1() {
    // 레디 사인 전송 안했으면 레디 사인 전송
    if (!sub_ready_sent) {
        // 서브 이용가능할때만 전송
        if (sub_available) {
            set_ready_to_sub();
        }

        sub_ready_sent = true;

        lcd.setCursor(0, 0);
        lcd.print("READY FOR START ");
        lcd.setCursor(0, 1);
        lcd.print("READY FOR START ");
        led_control(false, true, false);
    }

    // 초음파 센서로부터 거리 가져옴
    double distance = get_distance();

    // 거리 달라졌는지 확인
    if (is_distance_changed(distance)) {
        stopwatch_start = current_time();
        lcd_clear(0);
        mode = 2;
    }
}
void mode_2() {
    double time_now = current_time();
    // 스탑워치가 가동 중임.
    lcd.setCursor(0, 0);
    lcd.print("ELASPED:");
    lcd.print(time_now - stopwatch_start);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    led_control(false, false, true);

    // 초음파 센서로부터 거리 가져옴
    double distance = get_distance();

    // 거리 달라졌는지 확인, 3초 지났는지 확인
    if (is_distance_changed(distance) && time_now - stopwatch_start > 3) {
        // 임시 변수에 현재 시각 저장
        double tempTime = time_now;

        // 서브 통과 인증되지 않았으면 통과 확인
        if (sub_available && !sub_accepted) {
            lcd.setCursor(0, 0);
            lcd.print("CHECK SUB ACCEPT ");
            sub_accepted = read_accept_from_sub();
        }

        // 서브 통과 인증되었으면 종료
        if (sub_available && sub_accepted) {
            stopwatch_end = tempTime;
            stopwatch_advantage = 0;

            led_control(false, false, false);
            lcd_clear(0);
            touch_last_dist_change();

            mode = 3;
        }

    }
}

void mode_3() {
    // 결과 표시 모드

    // 초음파 센서로부터 거리 가져옴
    double distance = get_distance();

    // 거리 변경이 일어난 경우
    if (is_distance_changed(distance)) {
        change_saved_distance((int) distance);
        touch_last_dist_change();
    }

    // 10 센티미터 보다 가까이 있으면
    if (distance < 10) {
        // 1.25초동안 안정 상태였다면
        if (is_delayed_for(1.25)) {
            // 초기화
            lcd_clear(0);
            stable_wait_delay = 3;

            sub_accepted = false;
            sub_range_ready = false;
            sub_ready = false;
            sub_ready_sent = false;

            mode = 0;
        } else {
            // 안정화 시간 미달시 시간 표시
            lcd.setCursor(0, 1);
            lcd.print("RESET ");
            lcd.print(1.25 - (current_time() - last_dist_change));
            led_control(true, false, false);
        }
    } else {
        // 10센티보다 가까이 있지 않으면 결과 표시
        lcd.setCursor(0, 0);
        lcd.print(stopwatch_end - stopwatch_start - stopwatch_advantage);
        lcd.setCursor(0, 1);
        lcd.print("FINISHED     ");
        led_control(false, false, false);
    }

}

void loop() {
    // 각 모드에 맞는 함수 호출 
    
    if (mode == 0) {
        mode_0();
    }
    if (mode == 1) {
        mode_1();
    }
    if (mode == 2) {
        mode_2();
    }
    if (mode == 3) {
        mode_3();
    }

}
