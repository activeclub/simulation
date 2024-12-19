import { SchemaType } from "@google/generative-ai";

export class MultimodalLiveClient {
  public ws: WebSocket | null = null;
  public url = "";

  constructor({ apiKey }: { apiKey: string }) {
    this.url = `wss://generativelanguage.googleapis.com/ws/google.ai.generativelanguage.v1alpha.GenerativeService.BidiGenerateContent?key=${apiKey}`;
  }

  connect() {
    const ws = new WebSocket(this.url);

    ws.onerror = (event) => {
      console.error(event);
    };

    return new Promise<void>((resolve, reject) => {
      ws.onopen = (event) => {
        console.log("connected to socket:", event);
        this.ws = ws;

        this._send({
          setup: {
            model: "models/gemini-2.0-flash-exp",
            generationConfig: {
              responseModalities: "audio",
              speechConfig: {
                voiceConfig: { prebuiltVoiceConfig: { voiceName: "Aoede" } },
              },
            },
            systemInstruction: {
              parts: [
                {
                  text: 'You are my helpful assistant. Any time I ask you for a graph call the "render_altair" function I have provided you. Dont ask for additional information just make your best judgement.',
                },
              ],
            },
            tools: [
              // there is a free-tier quota for search
              { googleSearch: {} },
              {
                functionDeclarations: [
                  {
                    name: "render_altair",
                    description: "Displays an altair graph in json format.",
                    parameters: {
                      type: SchemaType.OBJECT,
                      properties: {
                        json_graph: {
                          type: SchemaType.STRING,
                          description:
                            "JSON STRING representation of the graph to render. Must be a string, not a json object",
                        },
                      },
                      required: ["json_graph"],
                    },
                  },
                ],
              },
            ],
          },
        });

        ws.onclose = (event) => {
          console.log("disconnected:", event);
          this.disconnect();
        };

        ws.onmessage = (event) => {
          console.log(event.data);
        };
      };
    });
  }

  disconnect() {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  send() {
    const message = {
      clientContent: {
        turns: [
          {
            role: "user",
            parts: [{ text: "Hello" }],
          },
        ],
      },
    };

    this._send(message);
  }

  protected _send(message: object) {
    if (!this.ws) {
      throw new Error("WebSocket is not connected");
    }
    this.ws.send(JSON.stringify(message));
  }
}
