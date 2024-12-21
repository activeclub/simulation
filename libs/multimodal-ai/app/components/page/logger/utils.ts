import type {
  ClientContentMessage,
  Interrupted,
  ModelTurn,
  RealtimeInputMessage,
  ServerContentMessage,
  SetupCompleteMessage,
  SetupMessage,
  ToolCallCancellationMessage,
  ToolCallMessage,
  ToolResponseMessage,
  TurnComplete,
} from "@/lib/multimodal-live-client";

// biome-ignore lint/suspicious/noExplicitAny: <explanation>
const prop = (a: any, prop: string, kind = "object") =>
  typeof a === "object" && typeof a[prop] === "object";

export const isSetupMessage = (a: unknown): a is SetupMessage =>
  prop(a, "setup");

export const isClientContentMessage = (a: unknown): a is ClientContentMessage =>
  prop(a, "clientContent");

export const isRealtimeInputMessage = (a: unknown): a is RealtimeInputMessage =>
  prop(a, "realtimeInput");

export const isToolResponseMessage = (a: unknown): a is ToolResponseMessage =>
  prop(a, "toolResponse");

// incoming messages
export const isSetupCompleteMessage = (a: unknown): a is SetupCompleteMessage =>
  prop(a, "setupComplete");

// biome-ignore lint/suspicious/noExplicitAny: <explanation>
export const isServerContenteMessage = (a: any): a is ServerContentMessage =>
  prop(a, "serverContent");

// biome-ignore lint/suspicious/noExplicitAny: <explanation>
export const isToolCallMessage = (a: any): a is ToolCallMessage =>
  prop(a, "toolCall");

export const isToolCallCancellation = (
  a: unknown,
): a is ToolCallCancellationMessage["toolCallCancellation"] =>
  // biome-ignore lint/suspicious/noExplicitAny: <explanation>
  typeof a === "object" && Array.isArray((a as any).ids);

export const isToolCallCancellationMessage = (
  a: unknown,
): a is ToolCallCancellationMessage =>
  prop(a, "toolCallCancellation") &&
  // biome-ignore lint/suspicious/noExplicitAny: <explanation>
  isToolCallCancellation((a as any).toolCallCancellation);

// biome-ignore lint/suspicious/noExplicitAny: <explanation>
export const isModelTurn = (a: any): a is ModelTurn =>
  typeof (a as ModelTurn).modelTurn === "object";

// biome-ignore lint/suspicious/noExplicitAny: <explanation>
export const isTurnComplete = (a: any): a is TurnComplete =>
  typeof (a as TurnComplete).turnComplete === "boolean";

// biome-ignore lint/suspicious/noExplicitAny: <explanation>
export const isInterrupted = (a: any): a is Interrupted =>
  (a as Interrupted).interrupted;
