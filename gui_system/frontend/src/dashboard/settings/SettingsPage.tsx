import { AdminLogin, BridgeSettings, NodeConfigPanel } from "./components";

export default function SettingsPage() {
  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
      <AdminLogin />
      <BridgeSettings />
      <NodeConfigPanel />
    </div>
  );
}
