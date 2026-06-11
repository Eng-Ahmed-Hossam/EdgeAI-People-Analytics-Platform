import { NextResponse } from "next/server";
import type { NextRequest } from "next/server";

// Routes that require an authenticated session.
// The proxy checks for cookie presence only (fast, no DB round-trip).
// An expired or invalid cookie bypasses this check; the AuthGuard client
// component handles that case by calling GET /api/auth/me and redirecting if
// the session is no longer valid.
const PROTECTED_PREFIXES = [
  "/workspace",
  "/compare",
  "/dashboard",
  "/report",
  "/account",
];

export function proxy(request: NextRequest) {
  const { pathname } = request.nextUrl;

  const isProtected = PROTECTED_PREFIXES.some(
    (p) => pathname === p || pathname.startsWith(p + "/")
  );

  if (isProtected && !request.cookies.get("access_token")) {
    const loginUrl = new URL("/login", request.url);
    loginUrl.searchParams.set("next", pathname);
    return NextResponse.redirect(loginUrl);
  }

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
