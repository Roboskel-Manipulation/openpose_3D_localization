#ifndef _BODY_25_POSE_OUTPUT_FORMAT_H_
#define _BODY_25_POSE_OUTPUT_FORMAT_H_

/* OpenPose headers */
#include <openpose/pose/poseParameters.hpp>

/* OpenPose BODY_25 Body Parts Index-to-Name Mapping */
const std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS
{
    {0,  "Nose"},
    {1,  "Neck"},
    {2,  "RShoulder"},
    {3,  "RElbow"},
    {4,  "RWrist"},
    {5,  "LShoulder"},
    {6,  "LElbow"},
    {7,  "LWrist"},
    {8,  "MidHip"},
    {9,  "RHip"},
    {10, "RKnee"},
    {11, "RAnkle"},
    {12, "LHip"},
    {13, "LKnee"},
    {14, "LAnkle"},
    {15, "REye"},
    {16, "LEye"},
    {17, "REar"},
    {18, "LEar"},
    {19, "LBigToe"},
    {20, "LSmallToe"},
    {21, "LHeel"},
    {22, "RBigToe"},
    {23, "RSmallToe"},
    {24, "RHeel"},
    {25, "Background"}
};

/* OpenPose BODY_25 Body Parts Name-to-Index Mapping */
const std::map<std::string, unsigned int> POSE_BODY_25_BODY_PARTS_NAME_TO_IDX
{
    {"Nose",        0},
    {"Neck",        1},
    {"RShoulder",   2},
    {"RElbow",      3},
    {"RWrist",      4},
    {"LShoulder",   5},
    {"LElbow",      6},
    {"LWrist",      7},
    {"MidHip",      8},
    {"RHip",        9},
    {"RKnee",       10},
    {"RAnkle",      11},
    {"LHip",        12},
    {"LKnee",       13},
    {"LAnkle",      14},
    {"REye",        15},
    {"LEye",        16},
    {"REar",        17},
    {"LEar",        18},
    {"LBigToe",     19},
    {"LSmallToe",   20},
    {"LHeel",       21},
    {"RBigToe",     22},
    {"RSmallToe",   23},
    {"RHeel",       24},
    {"Background",  25}
};

/* OpenPose BODY_25 Body Part Pairs Index-to-Index Mapping */
const std::map<unsigned int, unsigned int> POSE_BODY_25_BODY_PART_PAIRS
{
    {1, 8},
    {1, 2},
    {1, 5},
    {2, 3},
    {3, 4},
    {5, 6},
    {6, 7},
    {8, 9},
    {9, 10},
    {10, 11},
    {8, 12},
    {12, 13},
    {13, 14},
    {1, 0},
    {0, 15},
    {15, 17},
    {0, 16},
    {16, 18},
    {2, 17},
    {5, 18},
    {14, 19},
    {19, 20},
    {14, 21},
    {11, 22},
    {22, 23},
    {11, 24},
    {0, 1}  // reverse also, may prove useful
};

/* OpenPose BODY_25 Upper Body Part Pairs Predecessors Indexes-to-Index Mapping */
/* A pair's "predecessor" is the keypoint through which one of the pair's keypoints is immediately connected with the rest of the human body */
const std::map<unsigned int, unsigned int> POSE_BODY_25_UPPER_BODY_PART_PAIRS_PREDECESSORS
{
    /* for the pair {1, 8} */ {1, 0},
    /* for the pair {1, 2} */ {1, 8}, // {1, 0},
    /* for the pair {1, 5} */ // {1, 8},
    /* for the pair {2, 3} */ {2, 1},
    /* for the pair {3, 4} */ {3, 2},
    /* for the pair {5, 6} */ {5, 1},
    /* for the pair {6, 7} */ {6, 5},
    /* for the pair {8, 9} */ {8, 1},
    /* for the pair {8, 12} */ // {8, 1},
    /* for the pair {1, 0} */ // {1, 8},
    /* for the pair {0, 15} */ {0, 1},
    /* for the pair {15, 17} */ {15, 0},
    /* for the pair {0, 16} */ // {0, 1},
    /* for the pair {16, 18} */ {16, 0},
    /* for the pair {2, 17} */ {17, 15}, // {2, 1},
    /* for the pair {5, 18} */ {18, 16}, // {5, 1},
    /* for the pair {0, 1} */ {0, 15}, {0, 16} // {1, 8}
};

/* OpenPose BODY_25 Upper Body Parts Indexes vector */
const std::vector<unsigned int> POSE_BODY_25_UPPER_BODY_PARTS_IDX { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 15, 16, 17, 18 };

/* OpenPose BODY_25 Limbs Name-to-Indexes Mapping */
const std::map< std::string, std::vector<unsigned int> > POSE_BODY_25_LIMBS_NAME_TO_IDX
{
    {"Left Arm",   {1, 2, 3, 4} },
    {"Right Arm",  {1, 5, 6, 7} },
    {"Left Foot",   {9, 10, 11/*, 22, 23, 24*/} },
    {"Right Foot",  {12, 13, 14/*, 19, 20, 21*/} }
};

#endif