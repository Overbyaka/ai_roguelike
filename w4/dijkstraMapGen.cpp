#include "dijkstraMapGen.h"
#include "ecsTypes.h"
#include "dungeonUtils.h"

template<typename Callable>
static void query_dungeon_data(flecs::world &ecs, Callable c)
{
  static auto dungeonDataQuery = ecs.query<const DungeonData>();

  dungeonDataQuery.each(c);
}

template<typename Callable>
static void query_characters_positions(flecs::world &ecs, Callable c)
{
  static auto characterPositionQuery = ecs.query<const Position, const Team, const Hitpoints>();

  characterPositionQuery.each(c);
}

constexpr float invalid_tile_value = 1e5f;

static void init_tiles(std::vector<float> &map, const DungeonData &dd)
{
  map.resize(dd.width * dd.height);
  for (float &v : map)
    v = invalid_tile_value;
}

// scan version, could be implemented as Dijkstra version as well
static void process_dmap(std::vector<float> &map, const DungeonData &dd)
{
  bool done = false;
  auto getMapAt = [&](size_t x, size_t y, float def)
  {
    if (x < dd.width && y < dd.width && dd.tiles[y * dd.width + x] == dungeon::floor)
      return map[y * dd.width + x];
    return def;
  };
  auto getMinNei = [&](size_t x, size_t y)
  {
    float val = map[y * dd.width + x];
    val = std::min(val, getMapAt(x - 1, y + 0, val));
    val = std::min(val, getMapAt(x + 1, y + 0, val));
    val = std::min(val, getMapAt(x + 0, y - 1, val));
    val = std::min(val, getMapAt(x + 0, y + 1, val));
    return val;
  };
  while (!done)
  {
    done = true;
    for (size_t y = 0; y < dd.height; ++y)
      for (size_t x = 0; x < dd.width; ++x)
      {
        const size_t i = y * dd.width + x;
        if (dd.tiles[i] != dungeon::floor)
          continue;
        const float myVal = getMapAt(x, y, invalid_tile_value);
        const float minVal = getMinNei(x, y);
        if (minVal < myVal - 1.f)
        {
          map[i] = minVal + 1.f;
          done = false;
        }
      }
  }
}

void dmaps::gen_player_approach_map(flecs::world &ecs, std::vector<float> &map)
{
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    init_tiles(map, dd);
    query_characters_positions(ecs, [&](const Position& pos, const Team& t, const Hitpoints&)
    {
      if (t.team == 0) // player team hardcode
        map[pos.y * dd.width + pos.x] = 0.f;
    });
    process_dmap(map, dd);
  });
}

void dmaps::gen_melee_attack_map(flecs::world& ecs, const Team& team, std::vector<float>& map)
{
    query_dungeon_data(ecs, [&](const DungeonData& dd)
        {
            init_tiles(map, dd);
            query_characters_positions(ecs, [&](const Position& pos, const Team& t, const Hitpoints&)
                {
                    if (t.team != team.team)
                        map[pos.y * dd.width + pos.x] = 0.f;
                });
            process_dmap(map, dd);
        });
}

void dmaps::gen_wizard_attack_map(flecs::world& ecs, const Team& team, std::vector<float>& map)
{
    query_dungeon_data(ecs, [&](const DungeonData& dd)
        {
            init_tiles(map, dd);
            query_characters_positions(ecs, [&](const Position& pos, const Team& t, const Hitpoints&)
                {
                    if (t.team != team.team) {
                        int radius = 4;
                        for (int i = -radius; i <= radius; ++i) {
                            int j = radius - std::abs(i);
                            if (dungeon::is_tile_walkable(ecs, { pos.x + j, pos.y - i })) {
                                map[(pos.y - i) * dd.width + (pos.x + j)] = 0.f;
                            }
                            if (dungeon::is_tile_walkable(ecs, { pos.x - j, pos.y - i })) {
                                map[(pos.y - i) * dd.width + (pos.x - j)] = 0.f;
                            }
                        }
                    }
                });
            process_dmap(map, dd);
        });
}

void dmaps::gen_explore_map(flecs::world& ecs, std::vector<float>& map)
{
    query_dungeon_data(ecs, [&](const DungeonData& dd)
        {
            init_tiles(map, dd);
            static auto visibilityQuery = ecs.query<const DungeonVisibility>();
            visibilityQuery.each([&](const DungeonVisibility& dv) {
                for (int i = 0; i < dv.height; ++i) {
                    for (int j = 0; j < dv.width; ++j) {
                        if (!dv.tiles[i * dv.width + j]) {
                            map[i * dv.width + j] = 0.f;
                        }
                    }
                }
                });
            process_dmap(map, dd);
        });
}

void dmaps::gen_player_flee_map(flecs::world &ecs, std::vector<float> &map)
{
  gen_player_approach_map(ecs, map);
  for (float &v : map)
    if (v < invalid_tile_value)
      v *= -1.2f;
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    process_dmap(map, dd);
  });
}

void dmaps::gen_hive_pack_map(flecs::world &ecs, std::vector<float> &map)
{
  static auto hiveQuery = ecs.query<const Position, const Hive>();
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    init_tiles(map, dd);
    hiveQuery.each([&](const Position &pos, const Hive &)
    {
      map[pos.y * dd.width + pos.x] = 0.f;
    });
    process_dmap(map, dd);
  });
}

