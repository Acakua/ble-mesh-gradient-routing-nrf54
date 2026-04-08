# ADR: Redesign WSN Topology Dashboard to focus on RRT

## Context and Problem Statement
The current dashboard displays two panels: "Mạng RRT Hiện Tại (Raw)" and "Mạng SDN AI Tối Ưu (Dijkstra)". 
The user wants to remove the AI-predicted network display and only show the actual RRT network status to simplify the view and focus on the real-time physical topology.

## Design Decisions
1. **Frontend Layout**: 
   - Remove the dual-panel `network-container`.
   - Replace it with a single full-width container for the RRT network.
   - Update the header to reflect the single-view focus.
2. **Data Handling**:
   - The Gateway currently sends a JSON payload with `before` and `after` keys (both containing the same graph data).
   - The new `index.html` will be updated to only process the `payload.before` data (or `payload` if simplified).
   - I will keep the compatibility check in JS to handle both formats.
3. **Control Interface**:
   - Keep the context menu (Right-click/Double-click) for node identification and LED toggling.

## Consequences
- **Positive**: Cleaner UI, reduced browser resource usage (rendering one graph instead of two).
- **Negative**: The "AI Dijkstra" view will no longer be visible (as requested).
- **Neutral**: The backend `Gateway.py` can remain unchanged for now to maintain compatibility with other potential tools, but could be simplified later.

## Alternatives Considered
- **Toggle switch**: Add a toggle to switch between RRT and AI views. (Rejected: User specifically asked to NOT show the AI network).
- **Tabbed view**: Use tabs for RRT and AI. (Rejected: Same as above).
