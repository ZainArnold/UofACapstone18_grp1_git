
int buttonState12 = 0;
int buttonState11 = 0;
int buttonState10 = 0;
struct Bedroomstuff{
  int temp;
};
struct Bedroomstuff Bedroom[9];



void setup(){
  pinMode(12, INPUT);
  pinMode(11, INPUT);
  pinMode(10, INPUT);
  digitalWrite(12, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(10, HIGH);

  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
}

void loop(){
  buttonState12 = digitalRead(12);
  buttonState11 = digitalRead(11);
  buttonState10 = digitalRead(10);

  if (buttonState12 == LOW) {
    // turn LED on:
    digitalWrite(7, HIGH);
  } else {
    // turn LED off:
    digitalWrite(7, LOW);
  }
  
  if (buttonState11 == LOW) {
    // turn LED on:
    digitalWrite(6, HIGH);
  } else {
    // turn LED off:
    digitalWrite(6, LOW);
  }
  
  if (buttonState10 == LOW) {
    // turn LED on:
    digitalWrite(5, HIGH);
  } else {
    // turn LED off:
    digitalWrite(5, LOW);
  }




  
  for (int i = 0; i < 9; i++){
    Bedroom[i].temp += 1;
  }
}




