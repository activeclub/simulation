import type { LinksFunction, LoaderFunction } from "@remix-run/node";
import { Links, Meta, Outlet, Scripts, useLoaderData } from "@remix-run/react";

import { LiveAPIProvider } from "@/contexts/LiveApiContext";
import stylesheet from "@/tailwind.css?url";
import { Provider } from "jotai";

export const links: LinksFunction = () => [
  { rel: "stylesheet", href: stylesheet },
];

export const loader: LoaderFunction = () => {
  return Response.json({
    ENV: {
      GOOGLE_GEMINI_API_KEY: process.env.GOOGLE_GEMINI_API_KEY,
    },
  });
};

export default function App() {
  const data = useLoaderData<typeof loader>();

  return (
    <html lang="en">
      <head>
        <link rel="icon" href="data:image/x-icon;base64,AA" />
        <Meta />
        <Links />
      </head>
      <body>
        <Provider>
          <LiveAPIProvider apiKey={data.ENV.GOOGLE_GEMINI_API_KEY}>
            <Outlet />
          </LiveAPIProvider>
        </Provider>

        <script
          // biome-ignore lint/security/noDangerouslySetInnerHtml: <explanation>
          dangerouslySetInnerHTML={{
            __html: `window.ENV = ${JSON.stringify(data.ENV)}`,
          }}
        />
        <Scripts />
      </body>
    </html>
  );
}
