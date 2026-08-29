"""timdr_robot/bridges/opcua_stub.py — kontrakt integracji z OPC-UA.
================================================================================
Patrz docstring `bridges/__init__.py` po ogolny wzorzec. Prawdziwa
integracja wystawialaby wezel OPC-UA (np. `ns=2;s=TIMDR.<unit_id>.
<component_id>.Health`) aktualizowany przy kazdym `StatusEvent`. `asyncua`
(nastepca `python-opcua`) jest importowany DEFENSYWNIE - bez niego most
dziala w trybie dry-run.
"""
from __future__ import annotations

from typing import Dict, List

try:
    import asyncua  # type: ignore
    _IMPORT_ERROR = None
except Exception as e:  # pragma: no cover - zalezy od srodowiska
    asyncua = None
    _IMPORT_ERROR = str(e)

from ..status import StatusEvent


class OPCUABridge:
    """Kontrakt: `update_node(unit_id, event)` aktualizuje wezel OPC-UA
    odpowiadajacy jednemu komponentowi. Bez `asyncua` zainstalowanego
    dziala w trybie dry-run."""

    def __init__(self, endpoint: str = "opc.tcp://localhost:4840/timdr/server/", namespace: str = "TIMDR"):
        self.endpoint = endpoint
        self.namespace = namespace
        self.available = asyncua is not None
        self.import_error = _IMPORT_ERROR
        self.running = False
        self.node_values: Dict[str, Dict] = {}  # dry-run: symulowany "serwer" w pamieci
        self._server = None
        if self.available:  # pragma: no cover - wymaga zainstalowanego asyncua
            self._init_real_server()

    def _init_real_server(self) -> None:  # pragma: no cover - wymaga asyncua
        """TODO integracyjne: `asyncua.Server()`, `server.init()`,
        rejestracja namespace, start w petli asyncio. Celowo puste w tym
        szkielecie - NIE wystawia zadnego prawdziwego serwera OPC-UA."""
        pass

    def _node_id(self, unit_id: str, component_id: str) -> str:
        return f"ns=2;s={self.namespace}.{unit_id}.{component_id}.Health"

    def update_node(self, unit_id: str, event: StatusEvent) -> Dict:
        node_id = self._node_id(unit_id, event.axis_id)
        value = {"level": event.level.value, "message": event.message}
        self.node_values[node_id] = value
        if self.available and self.running:  # pragma: no cover - wymaga dzialajacego serwera
            self._update_real(node_id, value)
        return {"node_id": node_id, "value": value}

    def _update_real(self, node_id: str, value: Dict) -> None:  # pragma: no cover - wymaga asyncua
        """TODO integracyjne: `node = self._server.get_node(node_id)`,
        `await node.write_value(json.dumps(value))`."""
        pass
