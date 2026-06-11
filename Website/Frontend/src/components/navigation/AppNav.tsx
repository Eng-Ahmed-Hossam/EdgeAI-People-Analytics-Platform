"use client";

import Link from "next/link";
import { usePathname, useRouter } from "next/navigation";
import { useAuth } from "@/contexts/AuthContext";
import { cn } from "@/lib/utils";

interface NavItem {
  label: string;
  href: string;
  exact?: boolean;
}

const APP_LINKS: NavItem[] = [
  { label: "Explorer",  href: "/workspace", exact: false },
  { label: "Compare",   href: "/compare",   exact: false },
  { label: "Dashboard", href: "/dashboard", exact: false },
];

export function AppNav() {
  const pathname = usePathname();
  const router = useRouter();
  const { user, isLoading, logout } = useAuth();

  const isActive = (item: NavItem) =>
    item.exact ? pathname === item.href : pathname.startsWith(item.href);

  async function handleLogout() {
    await logout();
    router.push("/");
  }

  return (
    <header className="h-11 shrink-0 flex items-center justify-between px-5 bg-surface border-b border-hairline">
      {/* Brand */}
      <div className="flex items-center gap-2.5">
        <div
          className="w-5 h-5 rounded-(--r-sm) bg-accent flex items-center justify-center"
          aria-hidden="true"
        >
          <svg
            width="10"
            height="10"
            viewBox="0 0 10 10"
            fill="none"
            stroke="white"
            strokeWidth="1.5"
            strokeLinecap="round"
          >
            <circle cx="5" cy="4" r="2" />
            <path d="M1 9c0-2.2 1.8-4 4-4s4 1.8 4 4" />
          </svg>
        </div>
        <Link
          href="/"
          className="flex items-center gap-2 group"
          aria-label="EdgeAI Location Intelligence — home"
        >
          <span className="font-display text-sm font-semibold text-ink-strong tracking-tight">
            EdgeAI
          </span>
          <span className="font-body text-xs text-ink-faint">
            Location Intelligence
          </span>
        </Link>
      </div>

      {/* App navigation */}
      <nav className="flex items-center gap-1" aria-label="App navigation">
        {APP_LINKS.map((item) => (
          <Link
            key={item.href}
            href={item.href}
            aria-current={isActive(item) ? "page" : undefined}
            className={cn(
              "px-3 py-1.5 rounded-(--r-sm) font-body text-xs font-medium transition-colors",
              isActive(item)
                ? "bg-sunken text-ink-strong"
                : "text-ink-muted hover:text-ink-body hover:bg-sunken"
            )}
          >
            {item.label}
          </Link>
        ))}
      </nav>

      {/* Auth section — neutral during loading to avoid flash */}
      <div className="flex items-center gap-2 min-w-0">
        {isLoading ? (
          // Neutral placeholder — same visual weight as auth links, no content
          <span className="w-24 h-5 rounded-(--r-sm) bg-raised opacity-40" />
        ) : user ? (
          <>
            <Link
              href="/account"
              className={cn(
                "max-w-40 truncate text-xs font-body transition-colors",
                pathname.startsWith("/account")
                  ? "text-ink-strong"
                  : "text-ink-muted hover:text-ink-body"
              )}
              title={user.email}
            >
              {user.email}
            </Link>
            <button
              onClick={handleLogout}
              className="px-3 py-1.5 rounded-(--r-sm) font-body text-xs font-medium text-ink-muted hover:text-ink-body hover:bg-sunken transition-colors"
            >
              Sign out
            </button>
          </>
        ) : (
          <>
            <Link
              href="/login"
              className="px-3 py-1.5 rounded-(--r-sm) font-body text-xs font-medium text-ink-muted hover:text-ink-body hover:bg-sunken transition-colors"
            >
              Log in
            </Link>
            <Link
              href="/signup"
              className="px-3 py-1.5 rounded-(--r-sm) font-body text-xs font-medium bg-accent text-ink-strong hover:bg-accent-hover transition-colors"
            >
              Sign up
            </Link>
          </>
        )}
      </div>
    </header>
  );
}
