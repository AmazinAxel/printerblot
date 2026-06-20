#pragma once

inline void irq_set_exclusive_handler(unsigned int, void (*)()) {}
inline void irq_set_enabled(unsigned int, bool) {}
