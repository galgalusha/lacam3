#pragma once

struct Params {
  static bool FLG_SWAP;
  static bool FLG_STAR;
  static bool FLG_MULTI_THREAD;
  static int SCATTER_MARGIN;
  static int PIBT_NUM;
  static bool FLG_REFINER;
  static int REFINER_NUM;
  static bool RAND_LL_NODE;
  static bool EMPTY_LL_NODE;
  static bool ROLLOUT_MODE;
  static bool RAND_SCATTER_REMOVAL;
  static bool FLG_SCATTER;
  static float RANDOM_INSERT_PROB1;
  static float RANDOM_INSERT_PROB2;
  static bool FLG_RANDOM_INSERT_INIT_NODE;
  static float RECURSIVE_RATE;
  static double RECURSIVE_TIME_LIMIT;

  static bool SCATTER_MAB;
};
