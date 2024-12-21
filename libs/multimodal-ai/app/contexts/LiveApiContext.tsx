import {
  type FC,
  type PropsWithChildren,
  createContext,
  useContext,
} from "react";

import { type UseLiveAPIResults, useLiveAPI } from "@/hooks/use-live-api";

const LiveAPIContext = createContext<UseLiveAPIResults | undefined>(undefined);

export type LiveAPIProviderProps = PropsWithChildren<{
  apiKey: string;
}>;

export const LiveAPIProvider: FC<LiveAPIProviderProps> = ({
  apiKey,
  children,
}) => {
  const liveAPI = useLiveAPI({ apiKey });

  return (
    <LiveAPIContext.Provider value={liveAPI}>
      {children}
    </LiveAPIContext.Provider>
  );
};

export const useLiveAPIContext = () => {
  const context = useContext(LiveAPIContext);
  if (!context) {
    throw new Error("useLiveAPIContext must be used wihin a LiveAPIProvider");
  }
  return context;
};
