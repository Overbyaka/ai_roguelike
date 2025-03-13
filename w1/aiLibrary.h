#pragma once

#include "stateMachine.h"

// state machines
StateMachine *create_state_machine();

// states
State *create_sm_state(StateMachine *sm);
State *create_go_to_state(int x, int y);
State *create_eat_state();
State *create_sleep_state();
State *create_attack_enemy_state();
State *create_move_to_enemy_state();
State *create_move_to_player_state();
State *create_healing_state();
State *create_flee_from_enemy_state();
State *create_patrol_state(float patrol_dist);
State *create_nop_state();
State *create_spawn_slime_state();

// transitions
StateTransition *create_hunger_more_than_transition(int threshold);
StateTransition *create_fatigue_more_than_transition(int threshold);
StateTransition *create_enemy_available_transition(float dist);
StateTransition *create_enemy_reachable_transition();
StateTransition *create_ally_available_transition(float dist);
StateTransition *create_hitpoints_less_than_transition(float thres);
StateTransition *create_player_hitpoints_less_than_transition(float thres);
StateTransition *create_negate_transition(StateTransition *in);
StateTransition *create_and_transition(StateTransition *lhs, StateTransition *rhs);
StateTransition *create_always_transition();
StateTransition *create_dist_to_pos_transition(float dist, int x, int y);

// helpers
template<typename T>
T sqr(T a){ return a*a; }

template<typename T, typename U>
float dist_sq(const T &lhs, const U &rhs) { return float(sqr(lhs.x - rhs.x) + sqr(lhs.y - rhs.y)); }

template<typename T, typename U>
float dist(const T &lhs, const U &rhs) { return sqrtf(dist_sq(lhs, rhs)); }