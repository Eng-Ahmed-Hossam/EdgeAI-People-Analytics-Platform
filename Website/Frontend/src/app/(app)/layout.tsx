import { AppNav } from "@/components/navigation/AppNav";
import { AuthGuard } from "@/components/navigation/AuthGuard";
import type { ReactNode } from "react";

// Shell layout for all authenticated app routes:
//   /workspace, /compare, /dashboard, /account
//
// AppNav renders for everyone (shows loading state during session check).
// AuthGuard wraps the page content: shows a session-check indicator while
// the cookie is being validated, then redirects to /login if the session
// is invalid, or renders children once the user is confirmed.

export default function AppLayout({ children }: { children: ReactNode }) {
  return (
    <div className="flex h-screen flex-col overflow-hidden bg-page">
      <AppNav />
      <main className="flex-1 overflow-y-auto">
        <AuthGuard>{children}</AuthGuard>
      </main>
    </div>
  );
}
