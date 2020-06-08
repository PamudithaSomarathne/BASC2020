#define SPEED 100
#define INA 8
#define INB 9
#define EN 10
#define PWM 11
#define ENA 2
#define ENB 3

int count = 0;

void setup() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(ENA, INPUT);
  pinMode(ENB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENA),ISR0,CHANGE);
//  attachInterrupt(digitalPinToInterrupt(ENB),ISR1,RISING);

  digitalWrite(EN,LOW);
  digitalWrite(INA,LOW);
  digitalWrite(INB,LOW);
  analogWrite(PWM,0);

  Serial.begin(9600);
  Serial.println("Beginning testing...");

  digitalWrite(EN,HIGH);
}

void loop() {
  digitalWrite(INA,HIGH);
  digitalWrite(INB,LOW);
  analogWrite(PWM,SPEED);

  delay(2000);

  Serial.print("Count: ");
  Serial.println(count);

  analogWrite(PWM,0);

  delay(2000);
}

void ISR0(){
  count++;
}

//void ISR1(){
//  count++;
//}
