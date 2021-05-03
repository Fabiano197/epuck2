#include<bits/stdc++.h>

#define MAX_CORRELATION_ERROR 15
#define CLOSE_LOOP_DISTANCE 75
#define NB_LANDMARK_MAX 1000
#define NB_CORNERS_MAX 100
#define WALL 0

#define uint16_t unsigned int
#define int16_t int

using namespace std;

typedef struct {
	int16_t x; //mm
	int16_t y; //mm
} wall_t;

typedef struct {
  float alpha;
  float beta;
} line_t;

static uint16_t N_landmarks = 0;
static wall_t landmarks[NB_LANDMARK_MAX];
static uint16_t last_landmark_sent = 0;
static uint16_t N_corners = 0;
static wall_t corners[NB_CORNERS_MAX];



float distance(wall_t a, wall_t b){
  return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

void printLandmarks(wall_t* l_ptr, uint16_t l_size){
  for(uint16_t i = 0; i < l_size; i++){
    cout << l_ptr->x << " " << l_ptr->y << " " << 0 << endl;
    l_ptr++;
  }
}

line_t fitLine(wall_t* l_ptr_begin, uint16_t N){
  line_t line;
  float x_mean = 0, y_mean = 0, x_var = 0, xy_covar = 0;
  wall_t* l_ptr = l_ptr_begin;
  for(uint16_t i = 0; i < N; i++){
    x_mean+=l_ptr->x;
    y_mean+=l_ptr->y;
    l_ptr++;
  }
  x_mean/=N;
  y_mean/=N;
  l_ptr = l_ptr_begin;
  for(uint16_t i = 0; i < N; i++){
    x_var+=(l_ptr->x-x_mean)*(l_ptr->x-x_mean);
    xy_covar+= (l_ptr->x-x_mean)*(l_ptr->y-y_mean);
    l_ptr++;
  }
  line.beta = xy_covar/x_var;
  line.alpha = y_mean-line.beta*x_mean;
  return line;
}


line_t fitTwoPoints(wall_t a, wall_t b){
  line_t l;
  if(a.x==b.x){
    a.x++;
  }
  l.beta = (a.y-b.y)/(1.0*a.x-1.0*b.x);
  l.alpha = (b.y*a.x-a.y*b.x)/(1.0*a.x-1.0*b.x);
  return l;
}


float error(line_t line, wall_t l){
  return fabs(l.y-line.alpha-line.beta*l.x)/sqrt(line.beta*line.beta+1);
}


int16_t calculateMaxError(wall_t *l_ptr_begin, uint16_t N){
  uint16_t maxErrorIndex = 0;
  line_t line = fitTwoPoints(*l_ptr_begin, *(l_ptr_begin+N-1));
  for(uint16_t i = 0; i < N; i++){
    if(error(line, *(l_ptr_begin+i)) > error(line, *(l_ptr_begin+maxErrorIndex))){
      maxErrorIndex = i;
    }
  }
  if(error(line, *(l_ptr_begin+maxErrorIndex))< MAX_CORRELATION_ERROR)return -1;
  return maxErrorIndex;
}

wall_t calculateIntersection(line_t a, line_t b){
  wall_t l;
  if(a.beta==b.beta)a.beta*=0.0001;
  l.x = (b.alpha-a.alpha)/(a.beta-b.beta);
  l.y = a.beta*l.x+a.alpha;
  return l;
}

void calculateLineSegments(bool closeLoop){
  static line_t prevLine = {0, 0};
  static line_t firstLine = {0, 0};

  if(closeLoop){
    corners[N_corners] = calculateIntersection(prevLine, firstLine);
    corners[0] = corners[N_corners];
    N_corners++;
    return;
  }

  if(N_landmarks<2)return;
  int16_t devide = calculateMaxError(&landmarks[0], N_landmarks);
  if(devide < 2)return;
  if(prevLine.alpha == 0 && prevLine.beta == 0){
    prevLine = fitLine(&landmarks[0], devide);
    firstLine = prevLine;
    return;
  }
  line_t currentLine = fitLine(&landmarks[0], devide);
  corners[N_corners] = calculateIntersection(prevLine, currentLine);
  N_corners++;
  prevLine = currentLine;
  for(uint16_t i = 0; i < N_landmarks-devide-1; i++){
    landmarks[i] = landmarks[i+devide+1];
  }
  N_landmarks = N_landmarks-devide-1;
  last_landmark_sent = last_landmark_sent-devide-1;
  return;
}

vector<wall_t> filterData(vector<wall_t> &l, uint16_t N){
  vector<wall_t> fl;
  for(uint16_t i = 0; i < N/3; i++){
    wall_t tof, proximity, pos;
    tof = l[3*i];
    proximity = l[3*i+1];
    pos = l[3*i+2];
    if(distance(pos, proximity) < 100){
      fl.push_back(proximity);
    }
    if(distance(pos, tof) < 500){
      //fl.push_back(tof);
    }
  }
  return fl;
}

void simulateEpuck(vector<wall_t> &l, int N){
    for(int i = 0; i < N; i++){
      landmarks[N_landmarks] = l[i];
      N_landmarks++;
      if(true ){
          calculateLineSegments(false);
      }
      if(i > 30 && distance(l[0], l[i]) < CLOSE_LOOP_DISTANCE)break;
    }
    calculateLineSegments(false);
    calculateLineSegments(true);
}


int main(){
  int N, buffer;
  float buff;
  cin >> N;
  for(int i = 0; i < 5; i++)cin >> buff;
  vector<wall_t> l(N);
  vector<wall_t> line_segments;
  for(int i = 0; i < N; i++){
    cin >> l[i].x >> l[i].y >> buffer;
  }

  l = filterData(l, N);
  N = l.size();
  corners[0] = {0, 0};
  simulateEpuck(l, N);

  cout << N_landmarks << endl;
  cout << 0 << " " << 0 << " " << 0 << " " << 0.1 << " " << 0.1 << endl;
  printLandmarks(&l[0], N_landmarks);
  cout << N_corners << endl;
  printLandmarks(&corners[0], N_corners);
}
