#include <SoftwareSerial.h>
#include <math.h>
#include <Pixy2.h>
const float pi = 3.14159;

int position;
String trame;
SoftwareSerial mySerial(7, 8); // RX, TX


//--------------------------------------------------------------------------------Calculs de rotations--------------------------------------------------------------------------------   

//DONNEES DE DEPART
const float RobotX=162;
const float RobotY=8;

float DistanceDeuxPoints(float x1, float x2, float y1, float y2){
  return sqrt(pow((x2-x1),2)+pow((y2-y1),2));
}

float AngleMoteur0(float x,  float y){

  //POSITION OBJET
  float Bx=x; float By=y;
  
  //POSITION ANGLE DROIT 
  float Cx=Bx; float Cy=RobotY;
  
  float AB=DistanceDeuxPoints(Bx,RobotX,By,RobotY);
  float BC=DistanceDeuxPoints(Cx,Bx,Cy,By);
  float angle;
  if(Bx <= 160){angle = 175- (asin(BC/AB)*(180/pi));}
  else{angle = (asin(BC/AB)*(180/pi));}
  return map(angle,0,180,500,2500);
}

float AngleMoteur1(float ObjX, float ObjY){
  float a;
  float b = 18.5;
  float c = 14.6;
  float yc = (29.7*DistanceDeuxPoints(ObjX,RobotX,ObjY,RobotY))/164;
  a = sqrt(pow(5.1,2)+pow(yc,2));
  float angle;
  if(yc <= 12){
    angle = 180-((asin(5.1/a)*(180/pi)) + (acos((pow(b,2)+pow(a,2)-pow(c,2))/(2*a*b))*(180/pi)));
  }
  else{
    angle = 5+((asin(5.1/a)*(180/pi)) + (acos((pow(b,2)+pow(a,2)-pow(c,2))/(2*a*b))*(180/pi)));
  }
  return map(angle,0,180,500,2500);
} 

float AngleMoteur2(float ObjX, float ObjY){
  float a;
  float b = 18.5;
  float c = 14.6;
  float yc = (29.7*DistanceDeuxPoints(ObjX,RobotX,ObjY,RobotY))/164;
  a = sqrt(pow(5.1,2)+pow(yc,2));
  float angle = 170 - (acos((pow(b,2)+pow(c,2)-pow(a,2))/(2*b*c))*(180/pi));
  return map(angle,0,180,500,2500);
}

float AngleMoteur3(float ObjX, float ObjY){
  float b = 5.1;
  float yc = (29.7*DistanceDeuxPoints(ObjX,RobotX,ObjY,RobotY))/164;
  float angle = 110 - (atan(yc/b)*(180/pi));
  return map(angle,0,180,500,2500);
}

float FermerPince(){
  return 1500;
}

float OuvrirPince(){
  return 500;
}

//--------------------------------------------------------------------------------déplacements--------------------------------------------------------------------------------

Pixy2 pixy;

bool BougeMoteur(int id, float valeur){ //Moteur de 0 à 5
  int sensorValue = analogRead(A1);
  trame = "#" + String(id) + "P" + String(valeur) + " T1000";
  mySerial.println(trame);
  delay(500);
  return true;
} 

void PoseAttente(){
  BougeMoteur(0,1500);
  BougeMoteur(1,1900);
  BougeMoteur(2,2000);
  BougeMoteur(3,800);
}

void AttrapeObj(float ObjX, float ObjY){
  BougeMoteur(0,AngleMoteur0(ObjX,ObjY));
  BougeMoteur(2,1500);
  BougeMoteur(1,AngleMoteur1(ObjX,ObjY));
  BougeMoteur(2,AngleMoteur2(ObjX,ObjY));
  BougeMoteur(3,AngleMoteur3(ObjX,ObjY));
  BougeMoteur(4,FermerPince());
  PoseAttente();
}

void PoseObj(float ObjX,float ObjY){
  BougeMoteur(0,AngleMoteur0(ObjX,ObjY));
  BougeMoteur(2,1500);
  BougeMoteur(1,AngleMoteur1(ObjX,ObjY));
  BougeMoteur(2,AngleMoteur2(ObjX,ObjY));
  BougeMoteur(3,AngleMoteur3(ObjX,ObjY));
  BougeMoteur(4,OuvrirPince());
  BougeMoteur(2,1000);
  PoseAttente();
}


void TrieObj(int id){
  if(pixy.ccc.getBlocks() >= 1){
    switch(id){
      case 1: //vert en bas à gauche de la cam
        for(int i = 0; i < pixy.ccc.getBlocks(); i++){
          if(pixy.ccc.blocks[i].m_signature == id && (pixy.ccc.blocks[i].m_x >= 160 || pixy.ccc.blocks[i].m_y < 110)){
            AttrapeObj(pixy.ccc.blocks[i].m_x, pixy.ccc.blocks[i].m_y);
            PoseObj(90,150);
          }
        }
        break;
      case 2: //bleu en bas à droite de la cam
        for(int i = 0; i < pixy.ccc.getBlocks(); i++){
          if(pixy.ccc.blocks[i].m_signature == id && (pixy.ccc.blocks[i].m_x < 160 || pixy.ccc.blocks[i].m_y < 110)){
            AttrapeObj(pixy.ccc.blocks[i].m_x, pixy.ccc.blocks[i].m_y);
            PoseObj(240,150);
          }
        }
        break;
      case 3: //rouge en haut à gauche de la cam
        for(int i = 0; i < pixy.ccc.getBlocks(); i++){
          if(pixy.ccc.blocks[i].m_signature == id && (pixy.ccc.blocks[i].m_x >= 160 || pixy.ccc.blocks[i].m_y >= 110)){
            AttrapeObj(pixy.ccc.blocks[i].m_x, pixy.ccc.blocks[i].m_y);
            PoseObj(90,70);
          }
        }
        break;
      case 4: //jaune en haut à droite de la cam
        for(int i = 0; i < pixy.ccc.getBlocks(); i++){
          if(pixy.ccc.blocks[i].m_signature == id && (pixy.ccc.blocks[i].m_x < 160 || pixy.ccc.blocks[i].m_y >= 110)){
            AttrapeObj(pixy.ccc.blocks[i].m_x, pixy.ccc.blocks[i].m_y);
            PoseObj(240,70);
          }
        }
        break;
    }
  }
  
}

//-------------------------------------------------------------------------------------Main-------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.print("Starting...\n");
  pixy.init();
}

void loop(){
  PoseAttente();
  TrieObj(1);
  TrieObj(2);
  TrieObj(3);
  TrieObj(4);
}
