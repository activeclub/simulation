import { atom, useAtom } from "jotai";

import type { StreamingLog } from "./types";

const maxLogsAtom = atom(500);
const logsAtom = atom<StreamingLog[]>([]);
const logAtom = atom<null, StreamingLog[], void>(null, (get, set, args) => {
  const { date, type, message } = args;
  const prevLog = get(logsAtom).at(-1);
  if (prevLog && prevLog.type === type && prevLog.message === message) {
    set(logsAtom, [
      ...get(logsAtom).slice(0, -1),
      {
        date,
        type,
        message,
        count: prevLog.count ? prevLog.count + 1 : 1,
      } as StreamingLog,
    ]);
  } else {
    set(logsAtom, [
      ...get(logsAtom).slice(-(get(maxLogsAtom) - 1)),
      {
        date,
        type,
        message,
      } as StreamingLog,
    ]);
  }
});

export function useLoggerStore() {
  const [maxLogs] = useAtom(maxLogsAtom);
  const [logs, setLogs] = useAtom(logsAtom);
  const [, log] = useAtom(logAtom);

  return {
    maxLogs,
    logs,
    log,
    clearLogs: () => setLogs([]),
  };
}
