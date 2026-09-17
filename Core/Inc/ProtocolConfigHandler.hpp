#pragma once

#include "Bmp390.hpp"
#include "Mmc5983ma.hpp"
#include "UartProtocol.hpp"

namespace ProtocolConfigHandler {

[[nodiscard]] UartProtocol::AckPayload handleConfigCommand(const UartProtocol::ConfigCmdPayload& cmd,
                                                           Bmp390& bmp390, Mmc5983ma& mmc5983ma);

} // namespace ProtocolConfigHandler
