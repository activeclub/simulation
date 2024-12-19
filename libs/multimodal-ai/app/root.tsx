import { Links, Meta, Outlet, Scripts, useLoaderData } from "@remix-run/react";
import { useEffect, useMemo } from "react";
import { MultimodalLiveClient } from "./lib/multimodal-live-client";

export async function loader() {
  return Response.json({
    GOOGLE_GEMINI_API_KEY: process.env.GOOGLE_GEMINI_API_KEY,
  });
}

export default function App() {
  const { GOOGLE_GEMINI_API_KEY } = useLoaderData<typeof loader>();
  const client = useMemo(
    () => new MultimodalLiveClient({ apiKey: GOOGLE_GEMINI_API_KEY }),
    [GOOGLE_GEMINI_API_KEY],
  );

  useEffect(() => {
    client.connect();
    return () => {
      client.disconnect();
    };
  }, [client]);

  return (
    <html lang="en">
      <head>
        <link rel="icon" href="data:image/x-icon;base64,AA" />
        <Meta />
        <Links />
      </head>
      <body>
        <h1>Hello world!</h1>
        <button type="button" onClick={() => client.send()}>
          Send
        </button>
        <Outlet />

        <Scripts />
      </body>
    </html>
  );
}
