"use client";

import { useEffect, type ReactNode } from "react";
import { useRouter, usePathname } from "next/navigation";
import { useAuth } from "@/contexts/AuthContext";

/**
 * Client-side guard for the (app) layout group.
 *
 * The middleware (src/middleware.ts) handles the primary case: a request with
 * no session cookie is redirected to /login before React renders. This guard
 * handles the secondary case: the cookie exists but the JWT is expired or
 * invalid (getMe() returned 401), leaving user=null after the AuthProvider
 * resolves. In that case we redirect to /login?next=<current-path>.
 *
 * While the session is being verified (isLoading=true), a neutral loading
 * indicator is shown to prevent a flash of protected content.
 */
export function AuthGuard({ children }: { children: ReactNode }) {
  const { user, isLoading } = useAuth();
  const router = useRouter();
  const pathname = usePathname();

  useEffect(() => {
    if (!isLoading && !user) {
      router.replace(`/login?next=${encodeURIComponent(pathname)}`);
    }
  }, [isLoading, user, router, pathname]);

  if (isLoading) {
    return (
      <div className="flex h-full items-center justify-center bg-page">
        <span className="text-xs text-ink-faint font-mono tracking-wide">
          Verifying session…
        </span>
      </div>
    );
  }

  // Redirecting — render nothing to avoid a flash of content.
  if (!user) return null;

  return <>{children}</>;
}
