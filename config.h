#ifndef CONFIG_H
#define CONFIG_H
#include "tinyxml2.h"
#include "const.h"
#include <string>
#include <iostream>
#include <sstream>
#include <math.h>

class Config
{
public:
    Config();
    void getConfig(const char* fileName);
    double  precision;
    double  focal_weight;
    bool    use_cardinal;
    bool    use_disjoint_splitting;
    bool    max_biclique_overlap;
    int     hlh_type;
    int     connectdness;
    double  agent_size;
    double  timelimit;
};

#endif // CONFIG_H
