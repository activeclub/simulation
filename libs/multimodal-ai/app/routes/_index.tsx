import { Logger } from "@/components/page/logger";
import { Button } from "@/components/ui/button";
import { useLiveAPIContext } from "@/contexts/LiveApiContext";

export default function App() {
  const { connect, connected, disconnect } = useLiveAPIContext();

  return (
    <main className="flex">
      <section>
        <Logger filter="none" />
      </section>
      <section>
        <Button onClick={connected ? disconnect : connect}>
          {connected ? "Disconnect" : "Connect"}
        </Button>
      </section>
    </main>
  );
}
