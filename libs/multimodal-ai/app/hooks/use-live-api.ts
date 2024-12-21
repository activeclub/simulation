import { useCallback, useMemo, useState } from "react";

import { MultimodalLiveClient } from "@/lib/multimodal-live-client/multimodal-live-client";

export type UseLiveAPIResults = {
  client: MultimodalLiveClient;
  connected: boolean;
  connect: () => Promise<void>;
  disconnect: () => void;
};

export function useLiveAPI({ apiKey }: { apiKey: string }): UseLiveAPIResults {
  const client = useMemo(() => new MultimodalLiveClient({ apiKey }), [apiKey]);

  const [connected, setConnected] = useState(false);

  const connect = useCallback(async () => {
    client.disconnect();
    await client.connect();
    setConnected(true);
  }, [client]);

  const disconnect = useCallback(() => {
    client.disconnect();
    setConnected(false);
  }, [client]);

  return {
    client,
    connect,
    disconnect,
    connected,
  };
}
