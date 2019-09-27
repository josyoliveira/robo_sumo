int led = 13;
int buzzer = 11;
//--------------------------------------
// Chaves de seleção de funcao:
int chave1 = 1; // pino TX
int chave2 = 0; // pino RX
int chave3 = 2;
int chave4 = 12;

int x[5] = {0, 0, 0, 0, 0};
int j, achou;

//--------------------------------------
// Sensores de distancia:
int sensor1 = A0;
int sensor2 = A1;
int sensor4 = A4;
int sensor5 = A3;//A5; foi feita essa troca pq o pino A5 estava recebendo interferencia do A3 - 22/09/19
//int sensor7 = A3; // reserva

int estado_d, estado_fd, estado_fe, estado_e;
int ultima_busca = 0;
// calculo da media para as leituras de distancia:
int quant = 10;
int d_min_deteccao = 40;
//--------------------------------------
// Sensores de cor:
int sensor3 = A2;
int sensor6 = A6;
int cor_dir_sensor, cor_esq_sensor, cor_dir, cor_esq;
int defesa_lateral = 0;
//--------------------------------------
// motores:
int standby = 7; // pino para habilitar motores
int IN1A = 5;
int IN2A = 6;

int IN1B = 8;
int IN2B = 9;

int pinoPWM_velA = 3;
int pinoPWM_velB = 10;

int vel1 = 50;  // velocidade minima = 30
int vel2 = 100;
int vel3 = 150;
int vel4 = 200;
int vel5 = 250;
int m = 0;
int k = 1;
int h = 0, hh = 0;

int lateral = 0;

//--------------------------------------

unsigned long int tempo_para_inverter_rotacao = 0;

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0));

  pinMode(led, OUTPUT);
  pinMode(buzzer, OUTPUT);

  pinMode(chave1, INPUT);
  pinMode(chave2, INPUT);
  pinMode(chave3, INPUT);
  pinMode(chave4, INPUT);

  pinMode(standby, OUTPUT);

  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2B, OUTPUT);

  pinMode(pinoPWM_velA, OUTPUT);
  pinMode(pinoPWM_velB, OUTPUT);

  delay(3600);
}
void loop() {  //////////// void loop/////////void loop////////////////// //////////// void loop/////////void loop////////////////// //////////// void loop/////////void loop////////////////// //////////// void loop/////////void loop//////////////////

  // x[1] = digitalRead(chave1);
  //x[2] = digitalRead(chave2);
  x[3] = digitalRead(chave3);
  x[4] = digitalRead(chave4);

  if (x[3] == 0 && x[4] == 0) {
    parar(1);
    telasensores();
    //testeled(100);
  }
  else if (x[3] == 1 && x[4] == 0) {
    //direita(1, vel2, vel2)
    cor();
    //testebuzzerled();
    //teste_direcoes();
    //tornado_rampa();
    //busca_aleatoria_rampa();
    //tornado_lateral();
  }
  else if (x[3] == 0 && x[4] == 1) {
    //testebuzzerled();
    //teste_sensor_cor_e_buzzer();
    //descer_rampa();
    //parar_branco();
     tornado();
    //teste_sensor_cor_e_motores();
    //frente(1, vel2, vel2);
    //som();

    //parar(1);
  }
  else if (x[3] == 1 && x[4] == 1) {
    //distanciaEbuzzer();
    busca_aleatoria();
    //busca_aleatoria_lateral();
    //teste_sensor_cor_e_motores();
  }
}    //////////// void loop/////////////void loop////////////// //////////// void loop/////////void loop////////////////// //////////// void loop/////////void loop////////////////// //////////// void loop/////////void loop//////////////////

void testeled(int gg) {
  digitalWrite(led, HIGH);
  delay(gg);
  digitalWrite(led, LOW);
  delay(gg);
}


void testebuzzerled() {
  digitalWrite (led, HIGH); // Acende o led
  tone (buzzer, 262, 200); // Toca o buzzer em tonalidade de dó, em 262Hz
  delay (200); // Deixa o buzzer tocando em 262Hz, e o led ativado por 200 milésimos de segundos
  digitalWrite (led, LOW); // Apaga o led
  tone (buzzer, 330, 200); // Toca o buzzer em tonalidade de Mi, em 330Hz
  delay (200); // Tempo em que o led ficará apagado e o buzzer tocando em 330Hz.
}

void distanciaEbuzzer() {
  if (leitura_1_sensor(sensor1) < 20) {
    testebuzzerled();
  }
}

int leitura_1_sensor(int pin) {
  int  soma = 0, x;
  for (int i = 0; i < quant ; i++) {
    int sensorValue = analogRead(pin);
    int dist_cm = (6762 / (sensorValue - 9)) - 4;
    //dist_cm = pow(3027.4 / sensorValue, 1.2134);
    soma = soma + dist_cm;
    delay(1);
  }

  int distancia_media = soma / quant;
  /*
    if ((distancia_media > 0) && (distancia_media <= d_min_deteccao)) {
    x = 1;
    }
    else {
    x = 0;
    }
  */
  return (distancia_media);
}

int leitura_1_sensor_cm_media(int pin) {
  float soma = 0;
  float menor = 0, maior = 0;
  float distancia[quant];

  for (int i = 0; i < quant ; i++) {
    float sensorValue = analogRead(pin);
    float dist_cm = (6762 / (sensorValue - 9)) - 4;
    distancia[i] = dist_cm;

    //Serial.println(distancia[i]);
    //delay(200);

    soma = soma + distancia[i];
    //dist_cm = pow(3027.4 / sensorValue, 1.2134);
  }

  //delay(500);

  menor = distancia[0];

  for (int j = 0; j <= quant - 1 ; j++) {
    if (distancia[j] > maior) {
      maior = distancia[j];
    }
    if (distancia[j] < menor) {
      menor = distancia[j];
    }
  }

  soma = soma - maior - menor;

  float media = soma / (quant - 2);

  /*
    Serial.print("media = ");
    Serial.println(media);
    Serial.println();
    Serial.println();
  */

  return (media);
}

int leitura_1_sensor_com_media(int pin) {
  float soma = 0;
  float menor = 0, maior = 0;
  float distancia[quant];

  for (int i = 0; i < quant ; i++) {
    float sensorValue = analogRead(pin);
    float dist_cm = (6762 / (sensorValue - 9)) - 4;
    distancia[i] = dist_cm;

    //Serial.println(distancia[i]);
    //delay(200);

    soma = soma + distancia[i];
    //dist_cm = pow(3027.4 / sensorValue, 1.2134);
  }

  //delay(500);

  menor = distancia[0];

  for (int j = 0; j <= quant - 1 ; j++) {
    if (distancia[j] > maior) {
      maior = distancia[j];
    }
    if (distancia[j] < menor) {
      menor = distancia[j];
    }
  }

  soma = soma - maior - menor;

  float media = soma / (quant - 2);

  if ((media > 0) && (media <= d_min_deteccao)) {
    achou = 1;
  }
  else {
    achou = 0;
  }

  return (achou);
}


void cor() {
  cor_dir_sensor = analogRead(sensor3);
  cor_esq_sensor = analogRead(sensor6);

  /*
    Serial.print("cor_dir = ");
    Serial.print(cor_dir_sensor);
    Serial.print("  cor_esq = ");
    Serial.println(cor_esq_sensor);
  */

  // sensor de cor da direita:
  if (cor_dir_sensor > 740) {
    cor_dir = 1;    // preto
  }
  else {
    cor_dir = 0;  // branco
  }

  // sensor de cor da esquerda:
  if (cor_esq_sensor > 840) {
    cor_esq = 1;    // preto
  }
  else {
    cor_esq = 0;  // branco
  }


//  Serial.print("cor_dir = ");
//  Serial.print(cor_dir);
//  Serial.print("  cor_esq = ");
//  Serial.println(cor_esq);
//
//  delay(200);

}

void telasensores() {

  Serial.print("s1 = ");
  Serial.print(leitura_1_sensor_cm_media(sensor1));
  Serial.print(" cm | ");

  Serial.print("s2 = ");
  Serial.print(leitura_1_sensor_cm_media(sensor2));
  Serial.print(" cm | ");

  //Serial.print("s3 = ");
  //Serial.print(leitura_1_sensor_cm_media(sensor3));
  //Serial.print(" cm | ");

  Serial.print("s4 = ");
  Serial.print(leitura_1_sensor_cm_media(sensor4));
  Serial.print(" cm | ");

  Serial.print("s5 = ");
  Serial.print(leitura_1_sensor_cm_media(sensor5));
  Serial.print(" cm | ");

  //Serial.print("s6 = ");
  //Serial.print(leitura_1_sensor_cm_media(sensor6));
  //Serial.print(" cm | ");

  //Serial.print("s7 = ");
  //Serial.print(leitura_1_sensor_cm_media(sensor7));
  //Serial.print(" cm | ");

  Serial.println();

  delay(500);
}

void teste_sensor_cor_e_buzzer() {
  cor();
  if (cor_dir == 1 && cor_esq == 1) {
    tone (buzzer, 262, 200); // Toca o buzzer em tonalidade de dó, em 262Hz
    delay (200);
  }
  else {
    tone (buzzer, 440, 200); // Toca o buzzer em tonalidade de dó, em 262Hz
    delay (200);
  }
}

void descer_rampa() {
  frente(50, vel5, vel5);
  parar(10);
  re(80, vel5, vel5);
  parar(100);
}


void tornado() {
  

  if (hh == 0) {
    descer_rampa();
    hh = 1;
    esquerda(1, vel5, vel3);
  }

  cor();
  if (cor_dir == 1 && cor_esq == 1) {

    estado_d = leitura_1_sensor_com_media(sensor1);
    estado_fd = leitura_1_sensor_com_media(sensor2);
    estado_fe = leitura_1_sensor_com_media(sensor5);
    estado_e = leitura_1_sensor_com_media(sensor4);

    if (estado_d == 0 && estado_fd == 1 && estado_fe == 1 && estado_e == 0) {
      frente(1, vel5, vel5);
      //parar(1);
      digitalWrite(led, HIGH);
    }
    else if (estado_d == 0 && estado_fd == 0 && estado_fe == 0 && estado_e == 1) {
      ultima_busca = 0;
      esquerda(100, vel5, vel5);
      digitalWrite(led, HIGH);
    }
    else if (estado_d == 0 && estado_fd == 0 && estado_fe == 1 && estado_e == 0) {
      ultima_busca = 0;
      frente(1, vel5, vel4);
      digitalWrite(led, LOW);
    }
    else if (estado_d == 0 && estado_fd == 1 && estado_fe == 0 && estado_e == 0) {
      ultima_busca = 1;
      frente(1, vel4, vel5);
      digitalWrite(led, LOW);
    }
    else if (estado_d == 1 && estado_fd == 0 && estado_fe == 0 && estado_e == 0) {
      ultima_busca = 1;
      direita(100, vel5, vel5);
      digitalWrite(led, HIGH);
    }
    //else if (estado_d == 0 && estado_fd == 0 && estado_fe == 0 && estado_e == 0) {
    else {     
      mudarDirecaoDoGiro();
    }
  }
    else if (cor_dir == 0 && cor_esq == 1 && estado_e == 1) {
      direita(300, vel5, vel4);
      parar(10);
      re(300, vel4, vel4);
      parar(10);
    }
    else if (cor_dir == 1 && cor_esq == 0 && estado_d == 1) {
      esquerda(300, vel4, vel5);
      parar(10);
      re(300, vel4, vel4);
      parar(10);
    }
    else {
      parar(10);
      re(400, vel4, vel4);
      parar(10);
      esquerda(350, vel3, vel3);
     
      
    }
  }
 

void mudarDirecaoDoGiro(){

   
        if ((millis() - tempo_para_inverter_rotacao) >= 3000) {
        tempo_para_inverter_rotacao = millis();
          
          if(! digitalRead(IN2A)){
            esquerda(1, vel5, vel3);  
            }
           else{
            direita(1, vel3, vel5);
           } 
          
      }

  }

  void busca_aleatoria() {
    if (hh == 0) {
      descer_rampa();
      hh = 1;
    }
    cor();
    if (cor_dir == 1 && cor_esq == 1) {

      estado_d = leitura_1_sensor_com_media(sensor1);
      estado_fd = leitura_1_sensor_com_media(sensor2);
      estado_fe = leitura_1_sensor_com_media(sensor5);
      estado_e = leitura_1_sensor_com_media(sensor4);

      if (estado_d == 0 && estado_fd == 1 && estado_fe == 1 && estado_e == 0) {
        frente(1, vel5, vel5);
        //parar(1);
        digitalWrite(led, HIGH);
      }
      else if (estado_d == 0 && estado_fd == 0 && estado_fe == 0 && estado_e == 1) {
        ultima_busca = 0;
        esquerda(100, vel5, vel5);
        digitalWrite(led, HIGH);
      }
      else if (estado_d == 0 && estado_fd == 0 && estado_fe == 1 && estado_e == 0) {
        ultima_busca = 0;
        frente(1, vel5, vel4);
        digitalWrite(led, LOW);
      }
      else if (estado_d == 0 && estado_fd == 1 && estado_fe == 0 && estado_e == 0) {
        ultima_busca = 1;
        frente(1, vel4, vel5);
        digitalWrite(led, LOW);
      }
      else if (estado_d == 1 && estado_fd == 0 && estado_fe == 0 && estado_e == 0) {
        ultima_busca = 1;
        direita(100, vel5, vel5);
        digitalWrite(led, HIGH);
      }
      //else if (estado_d == 0 && estado_fd == 0 && estado_fe == 0 && estado_e == 0) {
      else {
        frente(1, vel2, vel2);
        digitalWrite(led, LOW);
      }
    }
    else if (cor_dir == 0 && cor_esq == 1 && estado_e == 1) {
      direita(300, vel5, vel4);
      parar(10);
      re(300, vel4, vel4);
      parar(10);
    }
    else if (cor_dir == 1 && cor_esq == 0 && estado_d == 1) {
      esquerda(300, vel4, vel5);
      parar(10);
      re(300, vel4, vel4);
      parar(10);
    }
    else {
      parar(10);
      re(200, vel4, vel4);
      parar(10);
      esquerda(350, vel3, vel3);
      parar(10);
    }
  }

  //*************************************************************************************
  void busca_aleatoria_lateral() {
    if (hh == 0) {
      descer_rampa();
      hh = 1;
    }
    cor();
    if (cor_dir == 1 && cor_esq == 1) {

      estado_d = leitura_1_sensor_com_media(sensor1);
      estado_fd = leitura_1_sensor_com_media(sensor2);
      estado_fe = leitura_1_sensor_com_media(sensor5);
      estado_e = leitura_1_sensor_com_media(sensor4);

      if (estado_d == 0 && estado_fd == 1 && estado_fe == 1 && estado_e == 0) {
        frente(1, vel5, vel5);
        digitalWrite(led, HIGH);
        lateral = 0;
      }
      else if (estado_d == 0 && estado_fd == 0 && estado_fe == 0 && estado_e == 1) {
        ultima_busca = 0;
        esquerda(1, vel4, vel4);
        digitalWrite(led, LOW);
        lateral++;
        if (lateral == 200) {
          digitalWrite(led, HIGH);
          //esquerda(200, vel5, vel5);
          direita(350, vel5, vel5);
          lateral = 0;
        }
      }
      else if (estado_d == 0 && estado_fd == 0 && estado_fe == 1 && estado_e == 0) {
        ultima_busca = 0;
        frente(1, vel5, vel4);
        digitalWrite(led, LOW);
        lateral = 0;
      }
      else if (estado_d == 0 && estado_fd == 1 && estado_fe == 0 && estado_e == 0) {
        ultima_busca = 1;
        frente(1, vel4, vel5);
        digitalWrite(led, LOW);
        lateral = 0;
      }
      else if (estado_d == 1 && estado_fd == 0 && estado_fe == 0 && estado_e == 0) {
        ultima_busca = 1;
        direita(1, vel4, vel4);
        digitalWrite(led, LOW);
        lateral++;
        if (lateral == 200) {
          digitalWrite(led, HIGH);
          esquerda(350, vel5, vel5);
          //direita(200, vel5, vel5);
          lateral = 0;
        }
      }
      //else if (estado_d == 0 && estado_fd == 0 && estado_fe == 0 && estado_e == 0) {
      else {
        frente(1, vel4, vel4);
        digitalWrite(led, LOW);
        lateral = 0;
      }
    } else if (cor_dir == 0 && cor_esq == 1 && estado_e == 1) {
      direita(300, vel5, vel4);
      parar(10);
      re(300, vel4, vel4);
      lateral = 0;
      digitalWrite(led, LOW);
      parar(10);
    }
    else if (cor_dir == 1 && cor_esq == 0 && estado_d == 1) {
      esquerda(300, vel4, vel5);
      parar(10);
      digitalWrite(led, LOW);
      re(300, vel4, vel4);
      lateral = 0;
      parar(10);
    }
    else {
      digitalWrite(led, LOW);
      lateral = 0;
      parar(10);
      re(200, vel4, vel4);
      parar(10);
      esquerda(350, vel3, vel3);
      parar(10);
    }
  }


  void parar_branco() {
    cor();
    if (cor_dir == 1 && cor_esq == 1) {
      frente(1, 60, 60);
    }
    else {
      //parar(1000);
      re(200,  vel4, vel4);
      parar(1000);
      re(500,  vel1, vel1);
    }
  }

  void frente(int tempo, int velA, int velB) {

    digitalWrite(standby, HIGH); //habilitar standby - motores on

    digitalWrite(IN1A, LOW);
    digitalWrite(IN2A, HIGH);
    digitalWrite(IN1B, HIGH);
    digitalWrite(IN2B, LOW);

    analogWrite(pinoPWM_velA, velA);
    analogWrite(pinoPWM_velB, velB);

    delay(tempo);
    //delayMicroseconds(tempo);
  }

  void direita(int tempo, int velA, int velB) {

    digitalWrite(standby, HIGH); //habilitar standby - motores on

    digitalWrite(IN1A, HIGH);
    digitalWrite(IN2A, LOW);
    digitalWrite(IN1B, HIGH);
    digitalWrite(IN2B, LOW);

    analogWrite(pinoPWM_velA, velA);
    analogWrite(pinoPWM_velB, velB);

    delay(tempo);
  }

  void esquerda(int tempo, int velA, int velB) {

    digitalWrite(standby, HIGH); //habilitar standby - motores on

    digitalWrite(IN1A, LOW);
    digitalWrite(IN2A, HIGH);
    digitalWrite(IN1B, LOW);
    digitalWrite(IN2B, HIGH);

    analogWrite(pinoPWM_velA, velA);
    analogWrite(pinoPWM_velB, velB);

    delay(tempo);
  }

  void re(int tempo, int velA, int velB) {

    digitalWrite(standby, HIGH); //habilitar standby - motores on

    digitalWrite(IN1A, HIGH);
    digitalWrite(IN2A, LOW);
    digitalWrite(IN1B, LOW);
    digitalWrite(IN2B, HIGH);

    analogWrite(pinoPWM_velA, velA);
    analogWrite(pinoPWM_velB, velB);

    delay(tempo);
  }

  void parar(int tempo) {

    digitalWrite(standby, LOW);
    delay(tempo);
  }








  void teste_sensor_cor_e_motores() {
    cor();
    if (cor_dir == 1 && cor_esq == 1) {
      frente(1, vel2, vel2);
    }
    else if (cor_dir == 0 && cor_esq == 1) {
      parar(10);
      re(400, vel4, vel2);
      parar(100);
      direita(400, vel5, vel5);
      parar(100);
    }
    else if (cor_dir == 1 && cor_esq == 0) {
      parar(10);
      re(400, vel2, vel4);
      parar(100);
      esquerda(400, vel5, vel5);
      parar(100);
    }
    else {
      m++;
      parar(10);
      re(400, vel3, vel3);
      parar(100);

      int aleatorio = random(2);     // imprime um número aleatório entre 0 e 1
      if (aleatorio == 0) {
        direita(400, vel3, vel3);
      }
      else {
        esquerda(400, vel3, vel3);
      }
      k = k + 1;
      parar(100);
    }
  }

  void teste_motor() {

    digitalWrite(standby, HIGH); //habilitar standby - motores on

    digitalWrite(IN1A, LOW);
    digitalWrite(IN2A, HIGH);
    digitalWrite(IN1B, HIGH);
    digitalWrite(IN2B, LOW);

    for (int i = 0; i < 50; i++) {
      Serial.print("vel motor = ");
      Serial.println(i);

      analogWrite(pinoPWM_velA, i);
      analogWrite(pinoPWM_velB, i);
      delay(200);
    }

    for (int i = 50; i > 0; i--) {
      Serial.print("vel motor = ");
      Serial.println(i);

      analogWrite(pinoPWM_velA, i);
      analogWrite(pinoPWM_velB, i);
      delay(200);
    }
  }
