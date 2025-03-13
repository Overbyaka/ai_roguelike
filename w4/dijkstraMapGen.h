#pragma once
#include <vector>
#include <flecs.h>
#include "ecsTypes.h"

namespace dmaps
{
  void gen_player_approach_map(flecs::world &ecs, std::vector<float> &map);
  void gen_player_flee_map(flecs::world &ecs, std::vector<float> &map);
  void gen_hive_pack_map(flecs::world &ecs, std::vector<float> &map);
  void gen_melee_attack_map(flecs::world& ecs, const Team& team, std::vector<float>& map);
  void gen_wizard_attack_map(flecs::world& ecs, const Team& team, std::vector<float>& map);
  void gen_explore_map(flecs::world& ecs, std::vector<float>& map);
};

