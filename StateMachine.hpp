#ifndef STATE_MACHINE_BASE_H
#define STATE_MACHINE_BASE_H

#include <unordered_map>
#include <functional>

namespace common{

template<typename Event,
         typename State>
class StateMachine {
public:
    struct StateHandler {
        State id;
        std::function<void()> onEnter;
        std::function<void()> onProcess;
        std::function<void()> onExit;
        
        StateHandler()
        : id()
        , onEnter(nullptr)
        , onProcess(nullptr)
        , onExit(nullptr)
        {}

        StateHandler( State _id,
                    std::function<void()> _enter,
                    std::function<void()> _process,
                    std::function<void()> _exit)
        : id(_id)
        , onEnter(_enter)
        , onProcess(_process)
        , onExit(_exit)
        {}

        StateHandler(const StateHandler& obj)
        : id(obj.id)
        , onEnter(obj.onEnter)
        , onProcess(obj.onProcess)
        , onExit(obj.onExit)
        {}

        StateHandler& operator=(const StateHandler& obj){
            id = obj.id;
            onEnter = obj.onEnter;
            onProcess = obj.onProcess;
            onExit = obj.onExit;
            return *this;
        }
    };

public:
    StateMachine() {};
    ~StateMachine() {};
    StateMachine(const StateMachine& classRef) = delete;
    StateMachine& operator=(const StateMachine&) = delete;
    
    void AddTransition(const State& current_st, const Event& trigger, const State& next_st);
    void AddState(const StateHandler& handler);
    bool SetInitialState(State state);

    void ProcessEvent(Event event);

    const Event& GetCurrentEvent() const;
    const State& GetCurrentState() const;
private:
    struct state_hash {
        template<typename T> size_t operator()(T t) const {
            return static_cast<size_t>(t);
        }
    };
    struct state_event_pair_hash {
        template<typename T, typename U> size_t operator()(const std::pair<T,U>& p) const {
            return (static_cast<size_t>(p.first) << 8) ^ static_cast<size_t>(p.second);
        }
    };

    Event                                                                   m_current_event;
    State                                                                   m_current_state_id;
    std::unordered_map<State, StateHandler, state_hash>                     m_states;
    std::unordered_map<std::pair<State,Event>, State, state_event_pair_hash>           m_transition_matrix;
};

template<typename Event, typename State>
void StateMachine<Event,State>::AddTransition(const State& current_st, const Event& trigger, const State& next_st) 
{
    m_transition_matrix[std::pair<State,Event>(current_st, trigger)] = next_st;
};

template<typename Event, typename State>
void StateMachine<Event,State>::AddState(const StateMachine<Event,State>::StateHandler& handler) 
{
    m_states[handler.id] = handler;
};

template<typename Event, typename State>
bool StateMachine<Event,State>::SetInitialState(State state) 
{
    bool result = false;
    
    if(m_states.end() != m_states.find(state))
    {
        m_current_state_id = state;
        m_states[state].onEnter();
    }

    return result;
}

template<typename Event, typename State>
void StateMachine<Event,State>::ProcessEvent(Event event) 
{
    m_current_event = event;
    const auto transition = m_transition_matrix.find(std::pair<State,Event>(m_current_state_id, event));

    if(m_transition_matrix.end() == transition){
      m_states[m_current_state_id].onProcess();
    } else {
      m_states[m_current_state_id].onExit();
      m_current_state_id = transition->second;
      m_states[m_current_state_id].onEnter();
    }
};

template<typename Event, typename State>
const Event& StateMachine<Event,State>::GetCurrentEvent() const 
{
    return m_current_event;
};

template<typename Event, typename State>
const State& StateMachine<Event,State>::GetCurrentState() const 
{
    return m_current_state_id;
};

}//namespace common



#endif //STATE_MACHINE_BASE_H