#include "roguelike.h"
#include "ecsTypes.h"
#include "raylib.h"
#include "stateMachine.h"
#include "aiLibrary.h"
#include "blackboard.h"
#include "math.h"
#include "dungeonUtils.h"
#include "dijkstraMapGen.h"
#include "dmapFollower.h"

static flecs::entity create_player_approacher(flecs::entity e)
{
  e.set(DmapWeights{{{"approach_map", {1.f, 1.f}}}});
  return e;
}

static flecs::entity create_player_fleer(flecs::entity e)
{
  e.set(DmapWeights{{{"flee_map", {1.f, 1.f}}}});
  return e;
}

static flecs::entity create_hive_follower(flecs::entity e)
{
  e.set(DmapWeights{{{"hive_map", {1.f, 1.f}}}});
  return e;
}

static flecs::entity create_blue_melee(flecs::entity e)
{
    e.set(DmapWeights{ {{"blue_melee_attack_map", {1.f, 1.f}}} })
        .set(Team{ 1 })
        .set(Color{ 0, 0, 255, 255 });
    return e;
}

static flecs::entity create_red_melee(flecs::entity e)
{
    e.set(DmapWeights{ {{"red_melee_attack_map", {1.f, 1.f}}} })
        .set(Team{ 2 })
        .set(Color{ 255, 0, 0, 255 });
    return e;
}

static void create_wizard_beh(flecs::entity e);

static flecs::entity create_blue_wizard(flecs::entity e)
{
    e.set(DmapWeights{ {{"blue_wizard_attack_map", {1.f, 1.f}}} })
        .set(Team{ 1 })
        .set(Color{ 0, 0, 255, 255 });
    create_wizard_beh(e);
    return e;
}

static flecs::entity create_red_wizard(flecs::entity e)
{
    e.set(DmapWeights{ {{"red_wizard_attack_map", {1.f, 1.f}}} })
        .set(Team{ 2 })
        .set(Color{ 255, 0, 0, 255 });
    create_wizard_beh(e);
    return e;
}

static flecs::entity create_hive_monster(flecs::entity e)
{
  e.set(DmapWeights{{{"hive_map", {1.f, 1.f}}, {"approach_map", {1.8, 0.8f}}}});
  return e;
}

static flecs::entity create_hive(flecs::entity e)
{
  e.add<Hive>();
  return e;
}


static void create_fuzzy_monster_beh(flecs::entity e)
{
  e.set(Blackboard{});
  BehNode *root =
    utility_selector({
      std::make_pair(
        sequence({
          find_enemy(e, 4.f, "flee_enemy"),
          flee(e, "flee_enemy")
        }),
        [](Blackboard &bb)
        {
          const float hp = bb.get<float>("hp");
          const float enemyDist = bb.get<float>("enemyDist");
          return (100.f - hp) * 5.f - 50.f * enemyDist;
        }
      ),
      std::make_pair(
        sequence({
          find_enemy(e, 3.f, "attack_enemy"),
          move_to_entity(e, "attack_enemy")
        }),
        [](Blackboard &bb)
        {
          const float enemyDist = bb.get<float>("enemyDist");
          return 100.f - 10.f * enemyDist;
        }
      ),
      std::make_pair(
        patrol(e, 2.f, "patrol_pos"),
        [](Blackboard &)
        {
          return 50.f;
        }
      ),
      std::make_pair(
        patch_up(100.f),
        [](Blackboard &bb)
        {
          const float hp = bb.get<float>("hp");
          return 140.f - hp;
        }
      )
    });
  e.add<WorldInfoGatherer>();
  e.set(BehaviourTree{root});
}

static void create_minotaur_beh(flecs::entity e)
{
  e.set(Blackboard{});
  BehNode *root =
    selector({
      sequence({
        is_low_hp(50.f),
        find_enemy(e, 4.f, "flee_enemy"),
        flee(e, "flee_enemy")
      }),
      sequence({
        find_enemy(e, 3.f, "attack_enemy"),
        move_to_entity(e, "attack_enemy")
      }),
      patrol(e, 2.f, "patrol_pos")
    });
  e.set(BehaviourTree{root});
}

static void create_wizard_beh(flecs::entity e)
{
    e.set(Blackboard{});
    BehNode* root =
        selector({
         try_attack_magic(4)
            });
    e.set(BehaviourTree{ root });
}

static void reveal_visibility(flecs::world& ecs, const Position& pos, DungeonVisibility& dv) {
    int radius = 2;
    for (int i = -radius; i <= radius; ++i) {
        for (int j = -radius; j <= radius; ++j) {
            if (dungeon::is_tile_walkable(ecs, Position{ pos.x + j, pos.y + i })) {
                dv.tiles[(pos.y + i) * dv.width + (pos.x + j)] = true;
            }
        }
    }
}

static Position find_free_dungeon_tile(flecs::world &ecs)
{
  static auto findMonstersQuery = ecs.query<const Position, const Hitpoints>();
  bool done = false;
  while (!done)
  {
    done = true;
    Position pos = dungeon::find_walkable_tile(ecs);
    findMonstersQuery.each([&](const Position &p, const Hitpoints&)
    {
      if (p == pos)
        done = false;
    });
    if (done)
      return pos;
  };
  return {0, 0};
}

static flecs::entity create_monster(flecs::world& ecs, Color col, const char* texture_src, Position pos = { -1, -1 })
{
  if (pos == Position{ -1, -1 }) {
    pos = find_free_dungeon_tile(ecs);
  }

  flecs::entity textureSrc = ecs.entity(texture_src);
  return ecs.entity()
    .set(Position{pos.x, pos.y})
    .set(MovePos{pos.x, pos.y})
    .set(Hitpoints{100.f})
    .set(Action{EA_NOP})
    .set(Color{col})
    .add<TextureSource>(textureSrc)
    .set(StateMachine{})
    .set(Team{1})
    .set(NumActions{1, 0})
    .set(MeleeDamage{20.f})
    .set(Blackboard{});
}

static void create_player(flecs::world &ecs, const char *texture_src)
{
  Position pos = find_free_dungeon_tile(ecs);

  flecs::entity textureSrc = ecs.entity(texture_src);
  ecs.entity("player")
    .set(Position{pos.x, pos.y})
    .set(MovePos{pos.x, pos.y})
    .set(Hitpoints{500.f})
    //.set(Color{0xee, 0xee, 0xee, 0xff})
    .set(Action{EA_NOP})
    .add<IsPlayer>()
    .set(Team{0})
    .set(PlayerInput{})
    .set(NumActions{2, 0})
    .set(Color{255, 255, 255, 255})
    .add<TextureSource>(textureSrc)
      .set(MeleeDamage{ 50.f });
}

static void create_blue_spawner(flecs::world& ecs)
{
    Position pos = { -1, -1 };
    static auto dungeonDataQuery = ecs.query<const DungeonData>();
    dungeonDataQuery.each([&](const DungeonData& dd)
        {
            do {
                pos = find_free_dungeon_tile(ecs);
            } while (pos.x > dd.width / 2);
        });

    ecs.entity()
        .set(Position{ pos.x, pos.y })
        .set(Color{ 0,0,255, 255 })
        .set(Spawner{ 10, 0, 3 })
        .set(Team{ 1 });
}

static void create_red_spawner(flecs::world& ecs)
{
    Position pos = { -1, -1 };
    static auto dungeonDataQuery = ecs.query<const DungeonData>();
    dungeonDataQuery.each([&](const DungeonData& dd)
        {
            do {
                pos = find_free_dungeon_tile(ecs);
            } while (pos.x < dd.width / 2);
        });

    ecs.entity()
        .set(Position{ pos.x, pos.y })
        .set(Color{ 255,0,0, 255 })
        .set(Spawner{ 10, 0, 3 })
        .set(Team{ 2 });
}

static void create_heal(flecs::world &ecs, int x, int y, float amount)
{
  ecs.entity()
    .set(Position{x, y})
    .set(HealAmount{amount})
    .set(Color{0xff, 0x44, 0x44, 0xff});
}

static void create_magic_ball(flecs::world& ecs, const Position& start_pos, const Position& end_pos, const Color& color)
{
    flecs::entity textureSrc = ecs.entity("magic_ball_tex");
    ecs.entity()
        .set(MagicBall{ start_pos, end_pos, 0 })
        .set(color)
        .add<TextureSource>(textureSrc);
}

static void create_powerup(flecs::world &ecs, int x, int y, float amount)
{
  ecs.entity()
    .set(Position{x, y})
    .set(PowerupAmount{amount})
    .set(Color{0xff, 0xff, 0x00, 0xff});
}

static void register_roguelike_systems(flecs::world &ecs)
{
  static auto dungeonDataQuery = ecs.query<const DungeonData>();
  ecs.system<PlayerInput, Action, const IsPlayer>()
    .each([&](PlayerInput &inp, Action &a, const IsPlayer)
    {
      bool left = IsKeyDown(KEY_LEFT);
      bool right = IsKeyDown(KEY_RIGHT);
      bool up = IsKeyDown(KEY_UP);
      bool down = IsKeyDown(KEY_DOWN);
      bool explore = IsKeyDown(KEY_ENTER);
      if (left && !inp.left)
        a.action = EA_MOVE_LEFT;
      if (right && !inp.right)
        a.action = EA_MOVE_RIGHT;
      if (up && !inp.up)
        a.action = EA_MOVE_UP;
      if (down && !inp.down)
        a.action = EA_MOVE_DOWN;
      if (explore && !inp.explore)
          a.action = EA_EXPLORE;
      inp.left = left;
      inp.right = right;
      inp.up = up;
      inp.down = down;
      inp.explore = explore;

      bool pass = IsKeyDown(KEY_SPACE);
      if (pass && !inp.passed)
        a.action = EA_PASS;
      inp.passed = pass;
    });
  ecs.system<const Position, const Color>()
    .with<TextureSource>(flecs::Wildcard)
    .with<BackgroundTile>()
    .each([&](flecs::entity e, const Position &pos, const Color color)
    {
      const auto textureSrc = e.target<TextureSource>();
      DrawTextureQuad(*textureSrc.get<Texture2D>(),
          Vector2{1, 1}, Vector2{0, 0},
          Rectangle{float(pos.x) * tile_size, float(pos.y) * tile_size, tile_size, tile_size}, color);
    });
  ecs.system<const Position, const Color>()
    .without<TextureSource>(flecs::Wildcard)
    .each([&](const Position &pos, const Color color)
    {
      const Rectangle rect = {float(pos.x) * tile_size, float(pos.y) * tile_size, tile_size, tile_size};
      DrawRectangleRec(rect, color);
    });
  ecs.system<const Position, const Color>()
    .with<TextureSource>(flecs::Wildcard)
    .without<BackgroundTile>()
    .each([&](flecs::entity e, const Position &pos, const Color color)
    {
      const auto textureSrc = e.target<TextureSource>();
      DrawTextureQuad(*textureSrc.get<Texture2D>(),
          Vector2{1, 1}, Vector2{0, 0},
          Rectangle{float(pos.x) * tile_size, float(pos.y) * tile_size, tile_size, tile_size}, color);
    });
  ecs.system<const Position, const Hitpoints>()
    .each([&](const Position &pos, const Hitpoints &hp)
    {
      constexpr float hpPadding = 0.05f;
      const float hpWidth = 1.f - 2.f * hpPadding;
      const Rectangle underRect = {float(pos.x + hpPadding) * tile_size, float(pos.y-0.25f) * tile_size,
                                   hpWidth * tile_size, 0.1f * tile_size};
      DrawRectangleRec(underRect, BLACK);
      const Rectangle hpRect = {float(pos.x + hpPadding) * tile_size, float(pos.y-0.25f) * tile_size,
                                hp.hitpoints / 100.f * hpWidth * tile_size, 0.1f * tile_size};
      DrawRectangleRec(hpRect, RED);
    });

  ecs.system<Texture2D>()
    .each([&](Texture2D &tex)
    {
      SetTextureFilter(tex, TEXTURE_FILTER_POINT);
    });
  ecs.system<const DmapWeights>()
    .with<VisualiseMap>()
    .each([&](const DmapWeights &wt)
    {
      dungeonDataQuery.each([&](const DungeonData &dd)
      {
        for (size_t y = 0; y < dd.height; ++y)
          for (size_t x = 0; x < dd.width; ++x)
          {
            float sum = 0.f;
            for (const auto &pair : wt.weights)
            {
              ecs.entity(pair.first.c_str()).get([&](const DijkstraMapData &dmap)
              {
                float v = dmap.map[y * dd.width + x];
                if (v < 1e5f)
                  sum += powf(v * pair.second.mult, pair.second.pow);
                else
                  sum += v;
              });
            }
            if (sum < 1e5f)
              DrawText(TextFormat("%.1f", sum),
                  (float(x) + 0.2f) * tile_size, (float(y) + 0.5f) * tile_size, 150, WHITE);
          }
      });
    });
  ecs.system<const DijkstraMapData>()
    .with<VisualiseMap>()
    .each([](const DijkstraMapData &dmap)
    {
      dungeonDataQuery.each([&](const DungeonData &dd)
      {
        for (size_t y = 0; y < dd.height; ++y)
          for (size_t x = 0; x < dd.width; ++x)
          {
            const float val = dmap.map[y * dd.width + x];
            if (val < 1e5f)
              DrawText(TextFormat("%.1f", val),
                  (float(x) + 0.2f) * tile_size, (float(y) + 0.5f) * tile_size, 150, WHITE);
          }
      });
    });
  ecs.system<MagicBall, const Color>()
      .with<TextureSource>(flecs::Wildcard)
      .each([&](flecs::entity e, MagicBall& ball, const Color& color) {
      if (ball.t >= 1.0f) {
          e.destruct();
      }
      else {
          ball.t = std::min(1.0f, ball.t + ecs.delta_time() * 5.f);
          float pos_x = (1 - ball.t) * ball.startPosition.x + ball.t * ball.endPosition.x;
          float pos_y = (1 - ball.t) * ball.startPosition.y + ball.t * ball.endPosition.y;

          const auto textureSrc = e.target<TextureSource>();
          DrawTextureQuad(*textureSrc.get<Texture2D>(),
              Vector2{ 1, 1 }, Vector2{ 0, 0 },
              Rectangle{ float(pos_x) * tile_size, float(pos_y) * tile_size, tile_size, tile_size }, color);
      }
          });

  ecs.system<const DungeonVisibility>()
      .each([&](flecs::entity e, const DungeonVisibility& dungeonVisibility)
          {
              const auto textureSrc = ecs.entity("floor_tex");
              for (int i = 0; i < dungeonVisibility.height; ++i) {
                  for (int j = 0; j < dungeonVisibility.width; ++j) {
                      if (!dungeonVisibility.tiles[i * dungeonVisibility.width + j]) {
                          DrawTextureQuad(*textureSrc.get<Texture2D>(),
                              Vector2{ 1, 1 }, Vector2{ 0, 0 },
                              Rectangle{ float(j) * tile_size, float(i) * tile_size, tile_size, tile_size }, Color{ 100, 100, 100, 255 });
                      }
                  }
              }
          });
}

void init_roguelike(flecs::world &ecs)
{
  register_roguelike_systems(ecs);

  ecs.entity("swordsman_tex")
    .set(Texture2D{LoadTexture("assets/swordsman.png")});
  ecs.entity("minotaur_tex")
    .set(Texture2D{LoadTexture("assets/minotaur.png")});
  ecs.entity("magic_ball_tex")
      .set(Texture2D{ LoadTexture("assets/pew.png") });
  ecs.entity("wizard_tex")
      .set(Texture2D{ LoadTexture("assets/wizard.png") });

  ecs.observer<Texture2D>()
    .event(flecs::OnRemove)
    .each([](Texture2D texture)
      {
        UnloadTexture(texture);
      });

  for (int i = 0; i < 3; ++i) {
      create_blue_spawner(ecs);
      create_red_spawner(ecs);
  }
  create_player(ecs, "swordsman_tex");

  ecs.entity("dungeon").insert([&](DungeonVisibility& dv) {
      reveal_visibility(ecs, *ecs.entity("player").get<Position>(), dv);
      });

  create_player(ecs, "swordsman_tex");

  ecs.entity("world")
    .set(TurnCounter{})
    .set(ActionLog{});
}

void init_dungeon(flecs::world &ecs, char *tiles, size_t w, size_t h)
{
  flecs::entity wallTex = ecs.entity("wall_tex")
    .set(Texture2D{LoadTexture("assets/wall.png")});
  flecs::entity floorTex = ecs.entity("floor_tex")
    .set(Texture2D{LoadTexture("assets/floor.png")});

  std::vector<char> dungeonData;
  dungeonData.resize(w * h);
  for (size_t y = 0; y < h; ++y)
    for (size_t x = 0; x < w; ++x)
      dungeonData[y * w + x] = tiles[y * w + x];
  std::vector<bool> dungeonVisibility;
  dungeonVisibility.resize(w * h);
  for (size_t y = 0; y < h; ++y)
      for (size_t x = 0; x < w; ++x)
          if (dungeonData[y * w + x] == dungeon::floor) {
              dungeonVisibility[y * w + x] = false;
          }
          else {
              dungeonVisibility[y * w + x] = true;
          }
  ecs.entity("dungeon")
      .set(DungeonData{dungeonData, w, h })
      .set(DungeonVisibility{dungeonVisibility, w, h });

  for (size_t y = 0; y < h; ++y)
    for (size_t x = 0; x < w; ++x)
    {
      char tile = tiles[y * w + x];
      flecs::entity tileEntity = ecs.entity()
        .add<BackgroundTile>()
        .set(Position{int(x), int(y)})
        .set(Color{255, 255, 255, 255});
      if (tile == dungeon::wall)
        tileEntity.add<TextureSource>(wallTex);
      else if (tile == dungeon::floor)
        tileEntity.add<TextureSource>(floorTex);
    }
}


static bool is_player_acted(flecs::world &ecs)
{
  static auto processPlayer = ecs.query<const IsPlayer, const Action>();
  bool playerActed = false;
  processPlayer.each([&](const IsPlayer, const Action &a)
  {
    playerActed = a.action != EA_NOP;
  });
  return playerActed;
}

static bool upd_player_actions_count(flecs::world &ecs)
{
  static auto updPlayerActions = ecs.query<const IsPlayer, NumActions>();
  bool actionsReached = false;
  updPlayerActions.each([&](const IsPlayer, NumActions &na)
  {
    na.curActions = (na.curActions + 1) % na.numActions;
    actionsReached |= na.curActions == 0;
  });
  return actionsReached;
}

static Position move_pos(Position pos, int action)
{
  if (action == EA_MOVE_LEFT)
    pos.x--;
  else if (action == EA_MOVE_RIGHT)
    pos.x++;
  else if (action == EA_MOVE_UP)
    pos.y--;
  else if (action == EA_MOVE_DOWN)
    pos.y++;
  return pos;
}

static void push_to_log(flecs::world &ecs, const char *msg)
{
  static auto queryLog = ecs.query<ActionLog, const TurnCounter>();
  queryLog.each([&](ActionLog &l, const TurnCounter &c)
  {
    l.log.push_back(std::to_string(c.count) + ": " + msg);
    if (l.log.size() > l.capacity)
      l.log.erase(l.log.begin());
  });
}

static Action playerExplore(flecs::world& ecs) {
    static auto playerExploreQuery = ecs.query<const IsPlayer, const Position>();
    float moveWeights[EA_MOVE_END];
    for (size_t i = 0; i < EA_MOVE_END; ++i)
        moveWeights[i] = 0.f;

    auto get_dmap_at = [&](const DijkstraMapData& dmap, const DungeonData& dd, size_t x, size_t y, float mult, float pow)
        {
            const float v = dmap.map[y * dd.width + x];
            if (v < 1e5f)
                return powf(v * mult, pow);
            return v;
        };

    ecs.each([&](const DungeonData& dd) {
        ecs.entity("explore_map").get([&](const DijkstraMapData& dmap) {
            playerExploreQuery.each([&](const IsPlayer&, const Position& pos) {
                moveWeights[EA_NOP] = get_dmap_at(dmap, dd, pos.x + 0, pos.y + 0, 1, 1);
                moveWeights[EA_MOVE_LEFT] = get_dmap_at(dmap, dd, pos.x - 1, pos.y + 0, 1, 1);
                moveWeights[EA_MOVE_RIGHT] = get_dmap_at(dmap, dd, pos.x + 1, pos.y + 0, 1, 1);
                moveWeights[EA_MOVE_UP] = get_dmap_at(dmap, dd, pos.x + 0, pos.y - 1, 1, 1);
                moveWeights[EA_MOVE_DOWN] = get_dmap_at(dmap, dd, pos.x + 0, pos.y + 1, 1, 1);
                });
            });
        });

    Action act;
    float minWt = moveWeights[EA_NOP];
    for (size_t i = 0; i < EA_MOVE_END; ++i)
        if (moveWeights[i] < minWt)
        {
            minWt = moveWeights[i];
            act.action = i;
        }
    return act;
}

static void process_actions(flecs::world &ecs)
{
  static auto processActions = ecs.query<Action, Position, MovePos, const MeleeDamage, const Team>();
  static auto processHeals = ecs.query<Action, Hitpoints>();
  static auto checkAttacks = ecs.query<const MovePos, Hitpoints, const Team>();
  static auto findMonstersQuery = ecs.query<const Position, const Hitpoints, const Team>();
  static auto processSpawn = ecs.query<const Team, Spawner, const Position>();
  static auto magicAttacks = ecs.query<MagicAttack, Action, const Position, const Team>();
  static auto processMagicBalls = ecs.query<MagicBall>();
  static auto dungeonVisibilityQuery = ecs.query<DungeonVisibility>();
  // Process all actions
  ecs.defer([&]
  {
          magicAttacks.each([&](MagicAttack& attack, Action& a, const Position& pos, const Team& team) {
              if (a.action == EA_ATTACK_MAGIC && attack.target.is_valid()) {
                  attack.target.insert([&](Hitpoints& hp, const Position& target_pos) {
                      hp.hitpoints -= attack.damage;
                      create_magic_ball(ecs, pos, target_pos, team.team == 1 ? Color{ 0, 0, 255, 255 } : Color{ 255, 0, 0, 255 });
                      });
                  a.action = EA_NOP;
              }
              });

          processSpawn.each([&](const Team& team, Spawner& spawner, const Position& pos)
              {
                  if (spawner.curSteps == 0) {
                      bool canSpawn = true;
                      int numBlue = 0;
                      int numRed = 0;
                      findMonstersQuery.each([&](const Position& p, const Hitpoints&, const Team& team)
                          {
                              if (p == pos)
                                  canSpawn = false;
                              if (team.team == 1) {
                                  numBlue += 1;
                              }
                              else if (team.team == 2) {
                                  numRed += 1;
                              }
                          });

                      if (canSpawn) {
                          if (team.team == 1 && numBlue <= spawner.maxUnits) {
                              const int randomNumber = rand() % 2;
                              if (randomNumber == 0) {
                                  create_blue_melee(create_monster(ecs, {}, "minotaur_tex", pos));
                              }
                              else {
                                  create_blue_wizard(create_monster(ecs, {}, "wizard_tex", pos));
                              }
                          }
                          else if (team.team == 2 && numRed <= spawner.maxUnits) {
                              const int randomNumber = rand() % 2;
                              if (randomNumber == 0) {
                                  create_red_melee(create_monster(ecs, {}, "minotaur_tex", pos));
                              }
                              else {
                                  create_red_wizard(create_monster(ecs, {}, "wizard_tex", pos));
                              }
                          }
                      }
                  }
                  spawner.curSteps = (spawner.curSteps + 1) % spawner.numSteps;
              });
    processHeals.each([&](Action &a, Hitpoints &hp)
    {
      if (a.action != EA_HEAL_SELF)
        return;
      a.action = EA_NOP;
      push_to_log(ecs, "Monster healed itself");
      hp.hitpoints += 10.f;

    });
    processActions.each([&](flecs::entity entity, Action &a, Position &pos, MovePos &mpos, const MeleeDamage &dmg, const Team &team)
    {
            if (entity.has<IsPlayer>()) {
                if (a.action == EA_EXPLORE) {
                    a = playerExplore(ecs);
                }
            }
      Position nextPos = move_pos(pos, a.action);
      bool blocked = !dungeon::is_tile_walkable(ecs, nextPos);
      checkAttacks.each([&](flecs::entity enemy, const MovePos &epos, Hitpoints &hp, const Team &enemy_team)
      {
        if (entity != enemy && epos == nextPos)
        {
          blocked = true;
          if (team.team != enemy_team.team)
          {
            push_to_log(ecs, "damaged entity");
            hp.hitpoints -= dmg.damage;
          }
        }
      });
      if (blocked)
        a.action = EA_NOP;
      else
        mpos = nextPos;
    });
    // now move
    processActions.each([&](flecs::entity entity, Action& a, Position& pos, MovePos& mpos, const MeleeDamage&, const Team&)
    {
            if (entity.has<IsPlayer>()) {
                dungeonVisibilityQuery.each([&](DungeonVisibility& dv) {
                    reveal_visibility(ecs, Position{ mpos.x, mpos.y }, dv);
                    });
            }
      pos = mpos;
      a.action = EA_NOP;
    });
  });

  static auto deleteAllDead = ecs.query<const Hitpoints>();
  ecs.defer([&]
  {
    deleteAllDead.each([&](flecs::entity entity, const Hitpoints &hp)
    {
      if (hp.hitpoints <= 0.f)
        entity.destruct();
    });
  });

  static auto playerPickup = ecs.query<const IsPlayer, const Position, Hitpoints, MeleeDamage>();
  static auto healPickup = ecs.query<const Position, const HealAmount>();
  static auto powerupPickup = ecs.query<const Position, const PowerupAmount>();
  ecs.defer([&]
  {
    playerPickup.each([&](const IsPlayer&, const Position &pos, Hitpoints &hp, MeleeDamage &dmg)
    {
      healPickup.each([&](flecs::entity entity, const Position &ppos, const HealAmount &amt)
      {
        if (pos == ppos)
        {
          hp.hitpoints += amt.amount;
          entity.destruct();
        }
      });
      powerupPickup.each([&](flecs::entity entity, const Position &ppos, const PowerupAmount &amt)
      {
        if (pos == ppos)
        {
          dmg.damage += amt.amount;
          entity.destruct();
        }
      });
    });
  });
}

template<typename T>
static void push_info_to_bb(Blackboard &bb, const char *name, const T &val)
{
  size_t idx = bb.regName<T>(name);
  bb.set(idx, val);
}

// sensors
static void gather_world_info(flecs::world &ecs)
{
  static auto gatherWorldInfo = ecs.query<Blackboard,
                                          const Position, const Hitpoints,
                                          const WorldInfoGatherer,
                                          const Team>();
  static auto alliesQuery = ecs.query<const Position, const Team>();
  gatherWorldInfo.each([&](Blackboard &bb, const Position &pos, const Hitpoints &hp,
                           WorldInfoGatherer, const Team &team)
  {
    // first gather all needed names (without cache)
    push_info_to_bb(bb, "hp", hp.hitpoints);
    float numAllies = 0; // note float
    float closestEnemyDist = 100.f;
    alliesQuery.each([&](const Position &apos, const Team &ateam)
    {
      constexpr float limitDist = 5.f;
      if (team.team == ateam.team && dist_sq(pos, apos) < sqr(limitDist))
        numAllies += 1.f;
      if (team.team != ateam.team)
      {
        const float enemyDist = dist(pos, apos);
        if (enemyDist < closestEnemyDist)
          closestEnemyDist = enemyDist;
      }
    });
    push_info_to_bb(bb, "alliesNum", numAllies);
    push_info_to_bb(bb, "enemyDist", closestEnemyDist);
  });
}

void process_turn(flecs::world &ecs)
{
  static auto stateMachineAct = ecs.query<StateMachine>();
  static auto behTreeUpdate = ecs.query<BehaviourTree, Blackboard>();
  static auto turnIncrementer = ecs.query<TurnCounter>();
  if (is_player_acted(ecs))
  {
    if (upd_player_actions_count(ecs))
    {
      // Plan action for NPCs
      gather_world_info(ecs);
      ecs.defer([&]
      {
              process_dmap_followers(ecs);
        stateMachineAct.each([&](flecs::entity e, StateMachine &sm)
        {
          sm.act(0.f, ecs, e);
        });
        behTreeUpdate.each([&](flecs::entity e, BehaviourTree &bt, Blackboard &bb)
        {
          bt.update(ecs, e, bb);
        });
      });
      turnIncrementer.each([](TurnCounter &tc) { tc.count++; });
    }
    process_actions(ecs);

    std::vector<float> approachMap;
    dmaps::gen_player_approach_map(ecs, approachMap);
    ecs.entity("approach_map")
      .set(DijkstraMapData{approachMap});

    std::vector<float> fleeMap;
    dmaps::gen_player_flee_map(ecs, fleeMap);
    ecs.entity("flee_map")
      .set(DijkstraMapData{fleeMap});

    std::vector<float> hiveMap;
    dmaps::gen_hive_pack_map(ecs, hiveMap);
    ecs.entity("hive_map")
      .set(DijkstraMapData{hiveMap});

    std::vector<float> blueMeleeAttackMap;
    dmaps::gen_melee_attack_map(ecs, { 1 }, blueMeleeAttackMap);
    ecs.entity("blue_melee_attack_map")
        .set(DijkstraMapData{ blueMeleeAttackMap });

    std::vector<float> redMeleeAttackMap;
    dmaps::gen_melee_attack_map(ecs, { 2 }, redMeleeAttackMap);
    ecs.entity("red_melee_attack_map")
        .set(DijkstraMapData{ redMeleeAttackMap });

    std::vector<float> blueWizardAttackMap;
    dmaps::gen_wizard_attack_map(ecs, { 1 }, blueWizardAttackMap);
    ecs.entity("blue_wizard_attack_map")
        .set(DijkstraMapData{ blueWizardAttackMap });

    std::vector<float> redWizardAttackMap;
    dmaps::gen_wizard_attack_map(ecs, { 2 }, redWizardAttackMap);
    ecs.entity("red_wizard_attack_map")
        .set(DijkstraMapData{ redWizardAttackMap });

    std::vector<float> exploreMap;
    dmaps::gen_explore_map(ecs, exploreMap);
    ecs.entity("explore_map")
        .set(DijkstraMapData{ exploreMap });

    //ecs.entity("flee_map").add<VisualiseMap>();
    ecs.entity("hive_follower_sum")
      //.set(DmapWeights{{{"flee_map", {1.f, 1.f}}}})
      .set(DmapWeights{{{"hive_map", {1.f, 1.f}}, {"approach_map", {1.8f, 0.8f}}}})
      .add<VisualiseMap>();
  }
}

void print_stats(flecs::world &ecs)
{
  static auto playerStatsQuery = ecs.query<const IsPlayer, const Hitpoints, const MeleeDamage>();
  playerStatsQuery.each([&](const IsPlayer &, const Hitpoints &hp, const MeleeDamage &dmg)
  {
    DrawText(TextFormat("hp: %d", int(hp.hitpoints)), 20, 20, 20, WHITE);
    DrawText(TextFormat("power: %d", int(dmg.damage)), 20, 40, 20, WHITE);
  });

  static auto actionLogQuery = ecs.query<const ActionLog>();
  actionLogQuery.each([&](const ActionLog &l)
  {
    int yPos = GetRenderHeight() - 20;
    for (const std::string &msg : l.log)
    {
      DrawText(msg.c_str(), 20, yPos, 20, WHITE);
      yPos -= 20;
    }
  });
}

