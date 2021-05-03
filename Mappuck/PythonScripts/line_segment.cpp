#include<bits/stdc++.h>

#define WALL -32768
#define MAX_WALL_ERROR 15
#define CLOSE_LOOP_DISTANCE 75

using namespace std;

typedef struct {
	int x; //mm
	int y; //mm
	int z; //mm, if z = WALL -> landmark is wall

} landmark_t;

typedef struct {
  float alpha;
  float beta;
} line_t;
static vector<landmark_t> corners;

float distance(landmark_t a, landmark_t b){
  return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

void printLandmarks(vector<landmark_t> &l, int N){
  for(int i = 0; i < N; i++){
    cout << l[i].x << " " << l[i].y << " " << l[i].z << endl;
  }
}

line_t linearRegression(vector<landmark_t> &l, int N){
  line_t line;
  float x_mean = 0, y_mean = 0, x_var = 0, xy_covar = 0;
  for(int i = 0; i < N; i++){
    x_mean+=l[i].x;
    y_mean+=l[i].y;
  }
  x_mean/=N;
  y_mean/=N;
  for(int i = 0; i < N; i++){
    x_var+=(l[i].x-x_mean)*(l[i].x-x_mean);
    xy_covar+= (l[i].x-x_mean)*(l[i].y-y_mean);
  }
  line.beta = xy_covar/x_var;
  line.alpha = y_mean-line.beta*x_mean;
  return line;
}

line_t fitTwoPoints(landmark_t a, landmark_t b){
  line_t l;
  if(a.x==b.x){
    a.x++;
  }
  l.beta = (a.y-b.y)/(1.0*a.x-1.0*b.x);
  l.alpha = (b.y*a.x-a.y*b.x)/(1.0*a.x-1.0*b.x);
  return l;
}

float error(line_t line, landmark_t l){
  return fabs(l.y-line.alpha-line.beta*l.x)/sqrt(line.beta*line.beta+1);
}

int maxError(vector<landmark_t> &l, int N){
  int maxErrorIndex = 0;
  line_t line = fitTwoPoints(l[0], l[N-1]);
  /*cout << "MAX error coord and line" << endl;
  cout << l[0].x << " " << l[0].y << " " << l[N-1].x << " " << l[N-1].y << endl;
  cout << line.alpha << " " << line.beta << endl;*/
  for(int i = 0; i < N; i++){
    if(error(line, l[i]) > error(line, l[maxErrorIndex])){
      maxErrorIndex = i;
    }
  }
  //cout << error(line, l[maxErrorIndex]) << endl;
  if(error(line, l[maxErrorIndex])< MAX_WALL_ERROR)return -1;
  return maxErrorIndex;
}

landmark_t calculateCorner(line_t a, line_t b){
  landmark_t l;
  if(a.beta==b.beta)a.beta*=0.0001;
  l.x = (b.alpha-a.alpha)/(a.beta-b.beta);
  l.y = a.beta*l.x+a.alpha;
  l.z = WALL;
  return l;
}

void createCorners(vector<landmark_t> &l, int N, bool isEnd){
  static line_t lastLine = {0, 0};
  static line_t firstLine = {0, 0};

  if(isEnd){
    corners.push_back(calculateCorner(lastLine, firstLine));
    corners[0] = corners[corners.size()-1];
    return;
  }
  if(l.size()<2)return;
  //cout << lastLine.alpha << " " << lastLine.beta << endl;
  int devide = maxError(l, N);
  /*cout << "DEVISION index+coordinates" << endl;
  cout << devide << endl;
  if(devide!=-1)cout << l[devide].x << " " << l[devide].y << endl;*/
  if(devide < 2)return;
  vector<landmark_t> back, front;
  for(int i = 0; i < N; i ++){
    if(i < devide)back.push_back(l[i]);
    if(i > devide)front.push_back(l[i]);
  }
  l = front;
  if(lastLine.alpha == 0 && lastLine.beta == 0){
    //cout << "Lin Reg coordinates" << endl;
    //printLandmarks(back, back.size());
    lastLine = linearRegression(back, back.size());
    firstLine = lastLine;
    /*cout << "First Lin Reg Line" << endl;
    cout << lastLine.alpha << " " << lastLine.beta << endl;*/
    return;
  }
  line_t currentLine = linearRegression(back, back.size());
  /*cout << "New Lin Reg Line" << endl;
  cout << currentLine.alpha << " " << currentLine.beta << endl;*/
  //cout << currentLine.alpha << " " << currentLine.beta << endl;
  corners.push_back(calculateCorner(lastLine, currentLine));
  lastLine = currentLine;
  return;
}

vector<landmark_t> filterData(vector<landmark_t> &l, int N){
  vector<landmark_t> fl;
  for(int i = 0; i < N/3; i++){
    landmark_t tof, proximity, pos;
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

void simulateEpuck(vector<landmark_t> &l, int N){
    vector<landmark_t> epuckLandmarks;
    for(int i = 0; i < N; i++){
      epuckLandmarks.push_back(l[i]);
      if(true ){
          /*cout << "Epuck Landmarks at time " << i << endl;
          printLandmarks(epuckLandmarks, epuckLandmarks.size());*/
          createCorners(epuckLandmarks, epuckLandmarks.size(), false);
      }
      if(i > 30 && distance(l[0], l[i]) < CLOSE_LOOP_DISTANCE)break;
    }
    /*cout << "Epuck Landmarks at time the end"<< endl;
    printLandmarks(epuckLandmarks, epuckLandmarks.size());*/
    createCorners(epuckLandmarks, epuckLandmarks.size(), false);
    createCorners(epuckLandmarks, epuckLandmarks.size(), true);
}


int main(){
  int N;
  float buff;
  cin >> N;
  for(int i = 0; i < 5; i++)cin >> buff;
  vector<landmark_t> l(N);
  vector<landmark_t> line_segments;
  for(int i = 0; i < N; i++){
    cin >> l[i].x >> l[i].y >> l[i].z;
  }

  l = filterData(l, N);
  N = l.size();
  corners.push_back({0, 0, WALL});
  simulateEpuck(l, N);

  cout << l.size() << endl;
  cout << 0 << " " << 0 << " " << 0 << " " << 0.1 << " " << 0.1 << endl;
  printLandmarks(l, l.size());
  cout << corners.size() << endl;
  printLandmarks(corners, corners.size());
}
