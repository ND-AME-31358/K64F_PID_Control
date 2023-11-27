
template <typename F, int N>
struct MovingAverageFilter {
  F data[N];
  int index;
  F result;
  F inv_N;
  inline MovingAverageFilter(F init_val){
    for (int i=0; i<N; i++){
      data[i] = init_val;
    }
    index = 0;
    result = 0.0;
    inv_N = 1.0/N;
  }
  inline F update(F val_in){
    result = result + (val_in - data[index]) * inv_N;
    data[index] = val_in;
    index++;
    if (index >= N) {index = 0;}
    return result;
  }
};