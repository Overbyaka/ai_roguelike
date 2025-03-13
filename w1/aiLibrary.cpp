#include "aiLibrary.h"
#include <flecs.h>
#include "ecsTypes.h"
#include "raylib.h"
#include "log.h"
#include <cfloat>
#include <cmath>

class AttackEnemyState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &/*ecs*/, flecs::entity entity) const override
  {
    entity.insert([&](Action &a)
    {
      a.action = EA_ATTACK;
    });
  }
};

class SMState : public State
{
  StateMachine *sm; // we own it
public:
  SMState(StateMachine *sm) : sm(sm) {}

  ~SMState() { delete sm; }

  void enter() const override
  {
    sm->switchState(0);
  }
  void exit() const override {}
  void act(float dt, flecs::world &ecs, flecs::entity entity) const override
  {
    sm->act(dt, ecs, entity);
  }
};

template<typename T, typename U>
static int move_towards(const T &from, const U &to)
{
  int deltaX = to.x - from.x;
  int deltaY = to.y - from.y;
  if (abs(deltaX) > abs(deltaY))
    return deltaX > 0 ? EA_MOVE_RIGHT : EA_MOVE_LEFT;
  return deltaY < 0 ? EA_MOVE_UP : EA_MOVE_DOWN;
}

static int inverse_move(int move)
{
  return move == EA_MOVE_LEFT ? EA_MOVE_RIGHT :
         move == EA_MOVE_RIGHT ? EA_MOVE_LEFT :
         move == EA_MOVE_UP ? EA_MOVE_DOWN :
         move == EA_MOVE_DOWN ? EA_MOVE_UP : move;
}


template<typename Callable>
static void on_closest_enemy_pos(flecs::world &ecs, flecs::entity entity, Callable c)
{
  static auto enemiesQuery = ecs.query<const Position, const Team>();
  entity.insert([&](const Position &pos, const Team &t, Action &a)
  {
    flecs::entity closestEnemy;
    float closestDist = FLT_MAX;
    Position closestPos;
    enemiesQuery.each([&](flecs::entity enemy, const Position &epos, const Team &et)
    {
      if (t.team == et.team)
        return;
      float curDist = dist(epos, pos);
      if (curDist < closestDist)
      {
        closestDist = curDist;
        closestPos = epos;
        closestEnemy = enemy;
      }
    });
    if (ecs.is_valid(closestEnemy))
      c(a, pos, closestPos);
  });
}

class MoveToEnemyState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &ecs, flecs::entity entity) const override
  {
    on_closest_enemy_pos(ecs, entity, [&](Action &a, const Position &pos, const Position &enemy_pos)
    {
      a.action = move_towards(pos, enemy_pos);
    });
  }
};

class MoveToPlayerState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float /* dt */, flecs::world &ecs, flecs::entity entity) const override
  {
    static auto playerQuery = ecs.query<const IsPlayer, const Position>();
    entity.insert([&](Action &a, const Position &pos)
    {
      playerQuery.each([&](const IsPlayer, const Position& player_pos)
      {
        a.action = move_towards(pos, player_pos);
      });
    });
  }
};

class HealingState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float /* dt */, flecs::world &ecs, flecs::entity entity) const override
  {
    debug("healing...");
    entity.insert([](Action &a)
    {
      a.action = EA_HEAL;
    });
  }
};

class FleeFromEnemyState : public State
{
public:
  FleeFromEnemyState() {}
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &ecs, flecs::entity entity) const override
  {
    on_closest_enemy_pos(ecs, entity, [&](Action &a, const Position &pos, const Position &enemy_pos)
    {
      a.action = inverse_move(move_towards(pos, enemy_pos));
    });
  }
};

class SpawnSlimeState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &ecs, flecs::entity entity) const override
  {
    entity.insert([](Action &a){
      a.action = EA_SPAWN;
    });
  }
};

class PatrolState : public State
{
  float patrolDist;
public:
  PatrolState(float dist) : patrolDist(dist) {}
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &ecs, flecs::entity entity) const override
  {
    entity.insert([&](const Position &pos, const PatrolPos &ppos, Action &a)
    {
      if (dist(pos, ppos) > patrolDist)
        a.action = move_towards(pos, ppos); // do a recovery walk
      else
      {
        // do a random walk
        a.action = GetRandomValue(EA_MOVE_START, EA_MOVE_END - 1);
      }
    });
  }
};

class GoToState : public State
{
  int x, y;
public:
  GoToState(int x, int y) : x(x), y(y) {}
  void enter() const override {}
  void exit() const override {}
  void act(float dt, flecs::world &ecs, flecs::entity entity) const override
  {
    entity.insert([&](const Position &pos, Action &a)
    {
      a.action = move_towards(pos, Position{x, y});
    });
  }
};

class EatState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float dt, flecs::world &ecs, flecs::entity entity) const override
  {
    if (!entity.has<Hunger>())
      return;

    entity.insert([](Hunger &hunger)
    {
      debug("eating...");
      hunger.current = 0;
    });
  }
};

class SleepState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float dt, flecs::world &ecs, flecs::entity entity) const override
  {
    if (!entity.has<Fatigue>())
      return;

    entity.insert([](Fatigue &fatigue)
    {
      debug("sleeping...");
      fatigue.current = 0;
    });
  }
};

class NopState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &ecs, flecs::entity entity) const override {}
};

class DistToPosTransition : public StateTransition
{
  float triggerDist;
  int x, y;
public:
  DistToPosTransition(float triggerDist, int x, int y)
    : triggerDist(triggerDist), x(x), y(y) {}
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    bool reached = false;
    entity.get([&](const Position &pos)
    {
      float curDist = dist(Position{x, y}, pos);
      reached |= curDist <= triggerDist;
    });
    return reached;
  }
};

template <typename Component>
class ComponentLessThanTransition : public StateTransition
{
  int threshold;
public:
  ComponentLessThanTransition(int threshold) : threshold(threshold) {}
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    bool reached = false;
    entity.get([&](const Component &comp)
    {
      reached |= comp.current <= threshold;
    });
    return reached;
  }
};

class EnemyAvailableTransition : public StateTransition
{
  float triggerDist;
public:
  EnemyAvailableTransition(float in_dist) : triggerDist(in_dist) {}
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    static auto enemiesQuery = ecs.query<const Position, const Team>();
    bool enemiesFound = false;
    entity.get([&](const Position &pos, const Team &t)
    {
      enemiesQuery.each([&](flecs::entity enemy, const Position &epos, const Team &et)
      {
        if (t.team == et.team)
          return;
        float curDist = dist(epos, pos);
        enemiesFound |= curDist <= triggerDist;
      });
    });
    return enemiesFound;
  }
};

class AllyAvailableTransition : public StateTransition
{
  float triggerDist;
public:
  AllyAvailableTransition(float in_dist) : triggerDist(in_dist) {}
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    static auto alliesQuery = ecs.query<const Position, const Team>();
    bool alliesFound = false;
    entity.get([&](const Position &pos, const Team &t)
    {
      alliesQuery.each([&](flecs::entity ally, const Position &apos, const Team at)
      {
        if (ally == entity)
          return;

        if (t.team != at.team)
          return;
        float curDist = dist(apos, pos);
        alliesFound |= curDist <= triggerDist;
      });
    });
    return alliesFound;
  }
};

class HitpointsLessThanTransition : public StateTransition
{
  float threshold;
public:
  HitpointsLessThanTransition(float in_thres) : threshold(in_thres) {}
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    bool hitpointsThresholdReached = false;
    entity.get([&](const Hitpoints &hp)
    {
      hitpointsThresholdReached |= hp.hitpoints < threshold;
    });
    return hitpointsThresholdReached;
  }
};

class PlayerHitpointsLessThanTransition : public StateTransition
{
  float threshold;
public:
  PlayerHitpointsLessThanTransition(float in_thres) : threshold(in_thres) {}
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    bool hitpointsThresholdReached = false;

    ecs.query<const IsPlayer, const Hitpoints>()
      .each([&](const IsPlayer, const Hitpoints hp)
    {
      hitpointsThresholdReached |= hp.hitpoints < threshold;
    });

    return hitpointsThresholdReached;
  }
};

class EnemyReachableTransition : public StateTransition
{
public:
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    return false;
  }
};

class NegateTransition : public StateTransition
{
  const StateTransition *transition; // we own it
public:
  NegateTransition(const StateTransition *in_trans) : transition(in_trans) {}
  ~NegateTransition() override { delete transition; }

  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    return !transition->isAvailable(ecs, entity);
  }
};

class AndTransition : public StateTransition
{
  const StateTransition *lhs; // we own it
  const StateTransition *rhs; // we own it
public:
  AndTransition(const StateTransition *in_lhs, const StateTransition *in_rhs) : lhs(in_lhs), rhs(in_rhs) {}
  ~AndTransition() override
  {
    delete lhs;
    delete rhs;
  }

  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    return lhs->isAvailable(ecs, entity) && rhs->isAvailable(ecs, entity);
  }
};

class AlwaysTransition : public StateTransition
{
public:
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    return true;
  }
};

// state machines
StateMachine *create_state_machine()
{
  return new StateMachine();
}

// states
State *create_sm_state(StateMachine *sm)
{
  return new SMState(sm);
}

State *create_go_to_state(int x, int y)
{
  return new GoToState(x, y);
}

State *create_eat_state()
{
  return new EatState();
}

State *create_sleep_state()
{
  return new SleepState();
}

State *create_attack_enemy_state()
{
  return new AttackEnemyState();
}
State *create_move_to_enemy_state()
{
  return new MoveToEnemyState();
}

State *create_move_to_player_state()
{
  return new MoveToPlayerState();
}

State *create_healing_state()
{
  return new HealingState();
}

State *create_flee_from_enemy_state()
{
  return new FleeFromEnemyState();
}


State *create_patrol_state(float patrol_dist)
{
  return new PatrolState(patrol_dist);
}

State *create_nop_state()
{
  return new NopState();
}

State *create_spawn_slime_state()
{
  return new SpawnSlimeState();
}

// transitions
StateTransition *create_hunger_more_than_transition(int threshold)
{
  return create_negate_transition(
    new ComponentLessThanTransition<Hunger>(threshold)
  );
}

StateTransition *create_fatigue_more_than_transition(int threshold)
{
  return create_negate_transition(
    new ComponentLessThanTransition<Fatigue>(threshold)
  );
}

StateTransition *create_dist_to_pos_transition(float dist, int x, int y)
{
  return new DistToPosTransition(dist, x, y);
}

StateTransition *create_enemy_available_transition(float dist)
{
  return new EnemyAvailableTransition(dist);
}

StateTransition *create_ally_available_transition(float dist)
{
  return new AllyAvailableTransition(dist);
}

StateTransition *create_enemy_reachable_transition()
{
  return new EnemyReachableTransition();
}

StateTransition *create_hitpoints_less_than_transition(float thres)
{
  return new HitpointsLessThanTransition(thres);
}

StateTransition *create_player_hitpoints_less_than_transition(float thres)
{
  return new PlayerHitpointsLessThanTransition(thres);
}

StateTransition *create_negate_transition(StateTransition *in)
{
  return new NegateTransition(in);
}
StateTransition *create_and_transition(StateTransition *lhs, StateTransition *rhs)
{
  return new AndTransition(lhs, rhs);
}

StateTransition *create_always_transition()
{
  return new AlwaysTransition();
}