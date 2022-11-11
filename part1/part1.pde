void setup(){
  fullScreen();
  surface.setTitle("Inverse Kinematics");
  
  //place objects and obstacles
  placeStuff();
  
  //initialize selected values
  initSelected();
  
  root = new Vec2(displayWidth/2,displayHeight/2);
}

//Root
Vec2 root = new Vec2(0,0);

//right arm
//Upper Arm
float lr = 75;
float ar = 0;

float l0r = 100; 
float a0r = 0.3; //Shoulder joint

//Lower Arm
float l1r = 85;
float a1r = 0.3; //Elbow joint

//Hand
float l2r = 25;
float a2r = 0.3; //Wrist joint

//Finger
float l3r = 25;
float a3r = 0.3; //Knuckle joint

Vec2 start_lr,start_l1r,start_l2r,start_l3r,endPointr;

//left arm
//Upper Arm
float ll = 75;
float al = 3.14;

float l0l = 100; 
float a0l = 3.14-0.3; //Shoulder joint

//Lower Arm
float l1l = 85;
float a1l = 3.14-0.3; //Elbow joint

//Hand
float l2l = 25;
float a2l = 3.14-0.3; //Wrist joint

//Finger
float l3l = 25;
float a3l = 3.14-0.3; //Knuckle joint

Vec2 start_ll,start_l1l,start_l2l,start_l3l,endPointl;

//object variables
static int maxNumObjects = 300;
int numCircs = 30;
int numRects = 30;
Vec2 circPos[] = new Vec2[maxNumObjects];
float circRad[] = new float[maxNumObjects];
Vec2 rectPos[] = new Vec2[maxNumObjects];
float rectWH[][] = new float[maxNumObjects][2];
boolean circSelected[] = new boolean[maxNumObjects];
boolean rectSelected[] = new boolean[maxNumObjects];

void solve(){
  Vec2 goal = new Vec2(mouseX,mouseY);
 
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  //right arm
  //Update knuckle joint
  startToGoal = goal.minus(start_l3r);
  startToEndEffector = endPointr.minus(start_l3r);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.12);
  if (cross(startToGoal,startToEndEffector) < 0)
    a3r += angleDiff;
  else
    a3r -= angleDiff;
  a3r = clamp(a3r,0,1.57);
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update wrist joint
  startToGoal = goal.minus(start_l2r);
  startToEndEffector = endPointr.minus(start_l2r);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.10);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2r += angleDiff;
  else
    a2r -= angleDiff;
  /*TODO: Wrist joint limits here*/
  a2r = clamp(a2r,-1.57,1.57);
  fk(); //Update link positions with fk (e.g. end effector changed)
   
   
  //Update elbow joint
  startToGoal = goal.minus(start_l1r);
  startToEndEffector = endPointr.minus(start_l1r);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.06);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1r += angleDiff;
  else
    a1r -= angleDiff;
  a1r = clamp(a1r,0,1.57);
  fk(); //Update link positions with fk (e.g. end effector changed)
   
   
  //Update shoulder joint
  startToGoal = goal.minus(start_lr);
  startToEndEffector = endPointr.minus(start_lr);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.05);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0r += angleDiff;
  else
    a0r -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  a0r = clamp(a0r,-1.57,1.57);
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPointr.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.05);
  if (cross(startToGoal,startToEndEffector) < 0)
    ar += angleDiff;
  else
    ar -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  ar = clamp(a0r,0,0);
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  //left arm
  //Update knuckle joint
  startToGoal = goal.minus(start_l3l);
  startToEndEffector = endPointl.minus(start_l3l);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.12);
  if (cross(startToGoal,startToEndEffector) < 0)
    a3l += angleDiff;
  else
    a3l -= angleDiff;
  a3l = clamp(a3l,-1.57,0);
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update wrist joint
  startToGoal = goal.minus(start_l2l);
  startToEndEffector = endPointl.minus(start_l2l);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.10);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2l += angleDiff;
  else
    a2l -= angleDiff;
  /*TODO: Wrist joint limits here*/
  a2l = clamp(a2l,-1.57,1.57);
  fk(); //Update link positions with fk (e.g. end effector changed)
   
   
  //Update elbow joint
  startToGoal = goal.minus(start_l1l);
  startToEndEffector = endPointl.minus(start_l1l);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.06);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1l += angleDiff;
  else
    a1l -= angleDiff;
  a1l = clamp(a1l,-1.57,0);
  fk(); //Update link positions with fk (e.g. end effector changed)
   
   
  //Update shoulder joint
  startToGoal = goal.minus(start_ll);
  startToEndEffector = endPointl.minus(start_ll);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.05);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0l += angleDiff;
  else
    a0l -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  a0l = clamp(a0l,-1.57,1.57);
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPointl.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = clamp(angleDiff,0,0.05);
  if (cross(startToGoal,startToEndEffector) < 0)
    al += angleDiff;
  else
    al -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  al = clamp(al,3.14,3.14);
  fk(); //Update link positions with fk (e.g. end effector changed)
}

void fk(){
  //right arm
  start_lr = new Vec2(cos(ar)*lr,sin(ar)*lr).plus(root);
  start_l1r = new Vec2(cos(ar+a0r)*l0r,sin(ar+a0r)*l0r).plus(start_lr);
  start_l2r = new Vec2(cos(ar+a0r+a1r)*l1r,sin(ar+a0r+a1r)*l1r).plus(start_l1r);
  start_l3r = new Vec2(cos(ar+a0r+a1r+a2r)*l2r,sin(ar+a0r+a1r+a2r)*l2r).plus(start_l2r);
  endPointr = new Vec2(cos(ar+a0r+a1r+a2r+a3r)*l3r,sin(ar+a0r+a1r+a2r+a3r)*l3r).plus(start_l3r);
  
  //left arm
  start_ll = new Vec2(cos(al)*ll,sin(al)*ll).plus(root);
  start_l1l = new Vec2(cos(al+a0l)*l0l,sin(al+a0l)*l0l).plus(start_ll);
  start_l2l = new Vec2(cos(al+a0l+a1l)*l1l,sin(al+a0l+a1l)*l1l).plus(start_l1l);
  start_l3l = new Vec2(cos(al+a0l+a1l+a2l)*l2l,sin(al+a0l+a1l+a2l)*l2l).plus(start_l2l);
  endPointl = new Vec2(cos(al+a0l+a1l+a2l+a3l)*l3l,sin(al+a0l+a1l+a2l+a3l)*l3l).plus(start_l3l);
}

float armW = 30;
void draw(){
  fk();
  solve();
  
  background(255,255,255);
  
  //draw a neck
  fill(161,102,94);
  pushMatrix();
  rect(root.x-armW/4,root.y-35,armW/2,15);
  popMatrix();
  
  //draw a head
  fill(161,102,94);
  pushMatrix();
  circle(root.x,root.y-63,30*2);
  popMatrix();
  
  //draw a body
  fill(161,102,94);
  pushMatrix();
  rect(root.x-armW/2,root.y-20,armW,200);
  popMatrix();
  
  //draw collar bone
  fill(161,102,94);
  pushMatrix();
  translate(root.x,root.y);
  rotate(ar);
  rect(0, -armW/2, lr, armW);
  popMatrix();
  
  fill(161,102,94);
  pushMatrix();
  translate(root.x,root.y);
  rotate(al);
  rect(0, -armW/2, ll, armW);
  popMatrix();
  
  //draw hips
  fill(161,102,94);
  pushMatrix();
  translate(root.x,root.y+180);
  rotate(ar);
  rect(0, -armW/2, 55, armW);
  popMatrix();
  
  fill(161,102,94);
  pushMatrix();
  translate(root.x,root.y+180);
  rotate(al);
  rect(0, -armW/2, 55, armW);
  popMatrix();
  
  //draw legs
  fill(161,102,94);
  pushMatrix();
  translate(root.x-55,root.y+180);
  rotate(1.57);
  rect(0, -armW/2, 200, armW);
  popMatrix();
  
  fill(161,102,94);
  pushMatrix();
  translate(root.x+55,root.y+180);
  rotate(1.57);
  rect(0, -armW/2, 200, armW);
  popMatrix();
  
  //draw random circles
  fill(255,0,0);
  for (int i = 0; i < numCircs; i++) {
    Vec2 c = circPos[i];
    float r = circRad[i];
    circle(c.x,c.y,r*2);
  }
  
  //draw random rectangles
  fill(0,0,255);
  for (int i = 0; i < numRects; i++) {
    Vec2 r = rectPos[i];
    float w = rectWH[i][0];
    float h = rectWH[i][1];
    rect(r.x,r.y,w,h);
  }

  //right arm
  fill(161,102,94);
  pushMatrix();
  translate(start_lr.x,start_lr.y);
  rotate(ar+a0r);
  rect(0, -armW/2, l0r, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1r.x,start_l1r.y);
  rotate(ar+a0r+a1r);
  rect(0, -armW/2, l1r, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2r.x,start_l2r.y);
  rotate(ar+a0r+a1r+a2r);
  rect(0, -armW/2, l2r, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l3r.x,start_l3r.y);
  rotate(ar+a0r+a1r+a2r+a3r);
  rect(0, -armW/2, l3r, armW);
  popMatrix();
  
  //left arm
  fill(161,102,94);
  pushMatrix();
  translate(start_ll.x,start_ll.y);
  rotate(al+a0l);
  rect(0, -armW/2, l0l, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1l.x,start_l1l.y);
  rotate(al+a0l+a1l);
  rect(0, -armW/2, l1l, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2l.x,start_l2l.y);
  rotate(al+a0l+a1l+a2l);
  rect(0, -armW/2, l2l, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l3l.x,start_l3l.y);
  rotate(al+a0l+a1l+a2l+a3l);
  rect(0, -armW/2, l3l, armW);
  popMatrix();
}

//Returns true iff the point 'point' is inside the box
boolean point_in_box(Vec2 point, Vec2 box_center, float box_w, float box_h, float box_angle){
  Vec2 relative_pos = point.minus(box_center);
  Vec2 box_right = new Vec2(cos(box_angle),sin(box_angle));
  Vec2 box_up = new Vec2(sin(box_angle),-cos(box_angle));
  float point_right = dot(relative_pos,box_right);
  float point_up = dot(relative_pos,box_up);
  if ((abs(point_right) < box_w/2) && (abs(point_up) < box_h/2))
    return true;
  return false;
}

boolean point_in_box_list(Vec2 point, Vec2[] rectPos, float[][] rectWH, int num) {
  for (int i = 0; i < num; i++) {
    if(rectPos[i] == null) return false;
    Vec2 center = new Vec2(rectPos[i].x+rectWH[i][0]/2, rectPos[i].y+rectWH[i][1]/2);
    float r = rectWH[i][0]/2 + 50;
    if (point_in_circle(center, r, point)) {
      return true;
    }
  }
  return false;
}

//Returns true iff the point 'point' is inside the circle
boolean point_in_circle(Vec2 center, float r, Vec2 pointPos){
  float dist = pointPos.distanceTo(center);
  if (dist < r+2){ //small safety factor
    return true;
  }
  return false;
}

boolean point_in_circle_list(Vec2[] centers, float[] radii, int num, Vec2 pointPos){
  for (int i = 0; i < num; i++){
    Vec2 center =  centers[i];
    if(center == null) return false;
    float r = radii[i] + 50;
    if (point_in_circle(center,r,pointPos)){
      return true;
    }
  }
  return false;
}

boolean point_in_skeleton(Vec2 pointPos) {
  root = new Vec2(displayWidth/2,displayHeight/2);
  if (pointPos.x > root.x-150 && pointPos.x < root.x+150 && pointPos.y > root.y-160 && pointPos.y < root.y+430) {
    return true;
  }
  return false;
}

//place objects and obstacles
void placeStuff() {
  for (int i = 0; i < numCircs; i++) {
    circPos[i] = sampleFreePos();
    circRad[i] = 30;
  }
  
  for (int i = 0; i < numRects; i++) {
    rectPos[i] = sampleFreePos();
    rectWH[i][0] = 60;
    rectWH[i][1] = 60;
  }
}

//get a random position that doesn't have anything inside of it
Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(0+0, displayWidth-60), random(0+60, displayHeight-60));
  boolean insideAnyCirc = point_in_circle_list(circPos, circRad, numCircs, randPos);
  boolean insideAnyRect = point_in_box_list(randPos, rectPos, rectWH, numRects);
  boolean insideSkeleton = point_in_skeleton(randPos);
  while (insideAnyCirc || insideAnyRect || insideSkeleton){
    randPos = new Vec2(random(0+0, displayWidth-60), random(0+60, displayHeight-60));
    insideAnyCirc = point_in_circle_list(circPos, circRad, numCircs, randPos);
    insideAnyRect = point_in_box_list(randPos, rectPos, rectWH, numRects);
    insideSkeleton = point_in_skeleton(randPos);
  }
  return randPos;
}

//create a reset button
void keyPressed() {
  if (key == 'r') {
    placeStuff();
  }
}

//create code for arm/object interaction
void mousePressed() {
  if (mouseButton == LEFT) {
    for (int i = 0; i < numCircs; i++) { 
       if (point_in_circle(circPos[i], circRad[i], endPointr)) {
         circSelected[i] = true;
       }
       if (point_in_circle(circPos[i], circRad[i], endPointl)) {
         circSelected[i] = true;
       }
    }
    for (int i = 0; i < numRects; i++) {
      Vec2 center = new Vec2(rectPos[i].x+rectWH[i][0]/2, rectPos[i].y+rectWH[i][1]/2);  
      if (point_in_box(endPointr, center, rectWH[i][0], rectWH[i][1], 0)) {
        rectSelected[i] = true;
      }
      if (point_in_box(endPointl, center, rectWH[i][0], rectWH[i][1], 0)) {
        rectSelected[i] = true;
      }
    }
  }
}

void mouseReleased() {
  initSelected();
}

void mouseDragged() {
  if (mouseButton == LEFT) {
    for (int i = 0; i < numCircs; i++) {
      if (circSelected[i] && point_in_circle(circPos[i], circRad[i], endPointr)) {
        circPos[i] = endPointr;
      }
      if (circSelected[i] && point_in_circle(circPos[i], circRad[i], endPointl)) {
        circPos[i] = endPointl;
      }
    }
    for (int i = 0; i < numRects; i++) {
      Vec2 center = new Vec2(rectPos[i].x+rectWH[i][0]/2, rectPos[i].y+rectWH[i][1]/2);  
      if (rectSelected[i] == true && point_in_box(endPointr, center, rectWH[i][0], rectWH[i][1], 0)) {
        rectPos[i] = new Vec2(endPointr.x-rectWH[i][0]/2, endPointr.y-rectWH[i][1]/2);
      }
      if (rectSelected[i] == true && point_in_box(endPointl, center, rectWH[i][0], rectWH[i][1], 0)) {
        rectPos[i] = new Vec2(endPointl.x-rectWH[i][0]/2, endPointl.y-rectWH[i][1]/2);
      }
    }
  }
}

//initialize objects selected value to false
void initSelected() {
  for (int i = 0; i < numCircs; i++) {
    circSelected[i] = false;
  }
  for (int i = 0; i < numRects; i++) {
    rectSelected[i] = false;
  }
}

//-----------------
// Vector Library
//-----------------

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
