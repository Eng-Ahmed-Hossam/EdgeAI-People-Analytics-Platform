import { AppShell } from "@/components/shell/AppShell";
import type { ReactNode } from "react";

// Shell layout for all authenticated app routes:
//   /workspace, /compare, /dashboard, /account
//
// AppShell owns the premium navigation frame and keeps AuthGuard around the
// content. Locked/gated modules are visual only until their contracts exist.

export default function AppLayout({ children }: { children: ReactNode }) {
  return <AppShell>{children}</AppShell>;
}
