/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

extern float floor_count_frac;

extern float slow_mode_safe_xvel;
extern float slow_mode_safe_yvel;

extern float fast_mode_safe_xvel;
extern float fast_mode_safe_yvel;

extern float cnn_frontal_obstacle_threshold;
extern float heading_turn_rate;
extern float cornering_turn_rate;

extern int cnn_enabled;

extern float OUTER_BOUNDS;
extern float INNER_BOUNDS;
extern float SAFE_BOUNDS;

extern void our_avoider_init(void);
extern void our_avoider_periodic(void);

#endif

