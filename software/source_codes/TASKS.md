# Task List

## Phase 0 - Critical Safety and Stability
- [ ] 0.1 Lock down remote UART control (bms_viewer.py)
- [x] 0.2 Fix unsafe SerialHandler::vsend (SerialHandler/Src/serial_handler.cpp)
- [x] 0.3 Initialize SD filename pointer (SdCardHandler)
- [x] 0.4 Contactor dwell reset after disable (PowerHandler)
- [x] 0.5 Bound EEPROM error index (ErrorHandler)
- [x] 0.6 Remove heap allocation in ISR (BQ79616_handler.cpp)

## Phase 1 - SOC Accuracy and Persistence
- [x] 1.1 OCV confidence scoring + knees
- [x] 1.2 Drift compensation and idle offset
- [x] 1.3 SOC persistence strategy (journal vs rate limit)
- [x] 1.4 Centralize SOC tunables

## Phase 2 - Balancing and Charge Control
- [x] 2.1 Smooth cell selection algorithm
- [x] 2.2 Balance progress tracking
- [x] 2.3 dV trend lock refactor
- [x] 2.4 Centralize balancing tunables + constraints

## Phase 3 - Protection System
- [x] 3.1 Consolidate protection evaluation
- [x] 3.2 Severity thresholds and behavior mapping
- [x] 3.3 Enhanced protection logging

## Phase 4 - Logging, Concurrency, and Data Integrity
- [x] 4.1 SD safe-eject coordination
- [x] 4.2 Pack info thread safety
- [x] 4.3 Interface TX queue safety
- [ ] 4.4 bms_viewer data correctness (deferred by user)

## Phase 5 - bms_viewer Refactor (Optional)
- [ ] 5.1 Module split (server/parsers/templates)
- [ ] 5.2 Optional feature additions
