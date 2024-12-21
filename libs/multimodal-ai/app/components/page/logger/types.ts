import type {
  LiveIncomingMessage,
  LiveOutgoingMessage,
} from "@/lib/multimodal-live-client";

export type StreamingLog = {
  date: Date;
  type: string;
  count?: number;
  message: string | LiveOutgoingMessage | LiveIncomingMessage;
};

export type LoggerFilterType = "conversations" | "tools" | "none";
