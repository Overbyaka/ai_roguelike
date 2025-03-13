#pragma once

struct Position;
struct MovePos;

struct MovePos
{
  int x = 0;
  int y = 0;

  MovePos &operator=(const Position &rhs);
};

struct Position
{
  int x = 0;
  int y = 0;

  Position &operator=(const MovePos &rhs);
};

inline Position &Position::operator=(const MovePos &rhs)
{
  x = rhs.x;
  y = rhs.y;
  return *this;
}

inline MovePos &MovePos::operator=(const Position &rhs)
{
  x = rhs.x;
  y = rhs.y;
  return *this;
}

inline bool operator==(const Position &lhs, const Position &rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
inline bool operator==(const Position &lhs, const MovePos &rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
inline bool operator==(const MovePos &lhs, const MovePos &rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
inline bool operator==(const MovePos &lhs, const Position &rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }


struct PatrolPos
{
  int x = 0;
  int y = 0;
};

struct EatPos
{
  int x = 0;
  int y = 0;
};

struct CraftPos
{
  int x = 0;
  int y = 0;
};

struct BuyPos
{
  int x = 0;
  int y = 0;
};

struct SellPos
{
  int x = 0;
  int y = 0;
};

struct SleepPos
{
  int x = 0;
  int y = 0;
};

struct CrafterPoses
{
  BuyPos buy;
  SellPos sell;
  CraftPos craft;
  EatPos eat;
  SleepPos sleep;
};

struct Hitpoints
{
  float hitpoints = 10.f;
};

enum Actions
{
  EA_NOP = 0,
  EA_MOVE_START,
  EA_MOVE_LEFT = EA_MOVE_START,
  EA_MOVE_RIGHT,
  EA_MOVE_DOWN,
  EA_MOVE_UP,
  EA_MOVE_END,
  EA_ATTACK,
  EA_HEAL,
  EA_SPAWN,
  EA_NUM
};

enum Mobs
{
  MOB_HUMAN = 0,
  MOB_SLIME,
  MOB_ARCHER,
  MOB_HEALER,
  MOB_CRAFTER,
  MOB_NUM
};

struct Action
{
  int action = 0;
};

struct NumActions
{
  int numActions = 1;
  int curActions = 0;
};

struct MeleeDamage
{
  float damage = 2.f;
};

struct RangedDamage
{
  float damage = 10.f;
};

struct RangedAttackRange
{
  float range = 5.f;
};

struct HealAmount
{
  float amount = 0.f;
};

struct HealRange
{
  float range = 1.f;
};

struct HealCooldown
{
  int current = 10;
  int cooldown = 10;
};

struct Hunger
{
  int current = 0;
  int max = 5;
};

struct Fatigue
{
  int current = 0;
  int max = 10;
};

struct PowerupAmount
{
  float amount = 0.f;
};

struct PlayerInput
{
  bool left = false;
  bool right = false;
  bool up = false;
  bool down = false;
};

struct Symbol
{
  char symb;
};

struct IsPlayer {};

struct IsMob {};

struct IsPickup {};

struct MobType
{
  unsigned type = MOB_HUMAN;
};

struct Team
{
  int team = 0;
};

struct TextureSource {};

