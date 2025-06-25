#include "source_manager/source.hpp"

State Source::get_state()const {
        return current_state_;
    }

State Source::get_prestate() const {
        return previous_state_;
    }

bool Source::is_state_changed() const {
        return state_changed_;
    }