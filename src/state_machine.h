#include <iostream>
#include <string>
#include <map>
#include <vector>
using namespace std;

enum class State
{
  Ready,
  LaneKeep,
  LaneChangeLeft,
  LangeChangeRight
};

inline ostream& operator<<(ostream& os, const State& s)
{
  switch (s)
  {
  case State::Ready:
    os << "System is ready";
    break;
  case State::LaneKeep:
    os << "Keeping lane";
    break;
  case State::LaneChangeLeft: 
    os << "Changing to left lane";
    break;
  case State::LangeChangeRight:
    os << "Changing to right lane";
    break;
  }
  return os;
}

enum class Trigger
{
  CallDialed,
  HungUp,
  CallConnected,
  PlacedOnHold,
  TakenOffHold,
  LeftMessage,
  StopUsingPhone
};

inline ostream& operator<<(ostream& os, const Trigger& t)
{
  switch (t)
  {
  case Trigger::CallDialed:
    os << "call dialed";
    break;
  case Trigger::HungUp:
    os << "hung up";
    break;
  case Trigger::CallConnected:
    os << "call connected";
    break;
  case Trigger::PlacedOnHold:
    os << "placed on hold";
    break;
  case Trigger::TakenOffHold:
    os << "taken off hold";
    break;
  case Trigger::LeftMessage: 
    os << "left message";
    break;
  case Trigger::StopUsingPhone:
    os << "putting phone on hook";
    break;
  default: break;
  }
  return os;
}