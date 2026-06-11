import { NextResponse } from "next/server";
import type { NextRequest } from "next/server";

// Session cookies are set by the Railway backend (a different domain).
// The proxy runs on Vercel and cannot read cross-domain cookies, so a
// cookie-presence check here would always fail and redirect logged-in users
// back to /login on every navigation.
//
// All auth protection is handled client-side by AuthGuard (components/navigation/AuthGuard.tsx):
//   - While loading: shows "Verifying session…"
//   - On load: calls GET /api/auth/me (cross-domain, credentials:include) to restore session
//   - If not authenticated: redirects to /login?next=<current-path>
//
// This proxy exists as a structural placeholder (e.g. for future non-cookie
// checks, A/B routing, or header manipulation) but must not redirect based on
// cookie presence.
export function proxy(_request: NextRequest) {
  return NextResponse.next();
}

export const config = {
  matcher: [
    "/workspace/:path*",
    "/compare/:path*",
    "/dashboard/:path*",
    "/report/:path*",
    "/account/:path*",
  ],
};
