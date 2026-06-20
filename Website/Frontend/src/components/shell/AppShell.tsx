"use client";

import Link from "next/link";
import { usePathname, useRouter } from "next/navigation";
import {
  BarChart3,
  Bell,
  Building2,
  ChevronDown,
  CircleDollarSign,
  ClipboardList,
  Compass,
  CreditCard,
  FileText,
  Home,
  Layers3,
  Lock,
  Map,
  Package,
  Radar,
  Settings,
  ShieldCheck,
  Smartphone,
  TrendingUp,
  Users,
} from "lucide-react";
import { useState, type ReactNode } from "react";
import { AuthGuard } from "@/components/navigation/AuthGuard";
import { useAuth } from "@/contexts/AuthContext";
import { cn } from "@/lib/utils";

interface NavItem {
  label: string;
  href?: string;
  icon: React.ComponentType<{ className?: string }>;
  status?: "active" | "locked" | "gated";
  teaser?: string;
}

const navGroups: Array<{ label: string; items: NavItem[] }> = [
  {
    label: "Home",
    items: [{ label: "Dashboard", href: "/dashboard", icon: Home, status: "active" }],
  },
  {
    label: "Operate",
    items: [
      { label: "Sales", icon: CircleDollarSign, status: "locked", teaser: "Unlocks when the money path is connected." },
      { label: "Customers", icon: Users, status: "locked", teaser: "Unlocks when customer records are connected." },
      { label: "Inventory", icon: Package, status: "locked", teaser: "Unlocks when stock data is connected." },
      { label: "Expenses", icon: CreditCard, status: "locked", teaser: "Unlocks when spend capture is connected." },
    ],
  },
  {
    label: "Analyze",
    items: [
      { label: "Reports", href: "/report", icon: FileText, status: "gated", teaser: "Current report uses demo scenarios until artifact routing is approved." },
      { label: "Street Pulse", icon: Radar, status: "locked", teaser: "Activates when Mahal Sense devices or rollups are paired." },
    ],
  },
  {
    label: "Grow",
    items: [
      { label: "Intelligence Map", href: "/workspace", icon: Map, status: "active" },
      { label: "Expansion Planner", icon: TrendingUp, status: "locked", teaser: "Unlocks after scenario and ranking contracts are approved." },
      { label: "Feasibility", href: "/workspace", icon: ClipboardList, status: "gated", teaser: "Runs through the Intelligence Map; history route is next." },
      { label: "Valuation", href: "/workspace", icon: BarChart3, status: "gated", teaser: "Computed today; persistent valuation history is not approved yet." },
      { label: "Compare", href: "/compare", icon: Layers3, status: "active" },
    ],
  },
  {
    label: "Manage",
    items: [
      { label: "Team", icon: ShieldCheck, status: "locked", teaser: "Requires role and membership contracts." },
      { label: "Devices", icon: Smartphone, status: "locked", teaser: "Device fleet management ships with Street Pulse." },
      { label: "Settings", href: "/account", icon: Settings, status: "gated", teaser: "Account settings are available; tenant settings arrive after identity bridge." },
    ],
  },
];

function SidebarItem({
  item,
  active,
  onTeaser,
}: {
  item: NavItem;
  active: boolean;
  onTeaser: (item: NavItem) => void;
}) {
  const Icon = item.icon;
  const locked = item.status === "locked";
  const gated = item.status === "gated";
  const base = cn(
    "group flex h-9 w-full items-center gap-2.5 rounded-(--r-md) px-3 text-left",
    "font-body text-sm transition-colors",
    active
      ? "bg-raised text-ink-strong shadow-[inset_2px_0_0_var(--color-accent)]"
      : "text-ink-muted hover:bg-raised/70 hover:text-ink-body",
    locked && "text-ink-faint hover:text-ink-muted"
  );

  const content = (
    <>
      <Icon className="h-4 w-4 shrink-0" />
      <span className="min-w-0 flex-1 truncate">{item.label}</span>
      {locked && <Lock className="h-3.5 w-3.5 shrink-0 opacity-70" />}
      {gated && <span className="h-1.5 w-1.5 rounded-full bg-verdict-warn" aria-hidden="true" />}
    </>
  );

  if (item.href && !locked) {
    return (
      <Link href={item.href} className={base} aria-current={active ? "page" : undefined}>
        {content}
      </Link>
    );
  }

  return (
    <button type="button" className={base} onClick={() => onTeaser(item)}>
      {content}
    </button>
  );
}

function Sidebar({ onTeaser }: { onTeaser: (item: NavItem) => void }) {
  const pathname = usePathname();

  return (
    <aside className="hidden w-[var(--nav-width)] shrink-0 border-r border-hairline bg-surface lg:flex lg:flex-col">
      <div className="border-b border-hairline px-4 py-4">
        <Link href="/dashboard" className="flex items-center gap-3">
          <div className="flex h-9 w-9 items-center justify-center rounded-(--r-md) border border-accent/35 bg-accent-soft text-accent-muted">
            <Compass className="h-4 w-4" />
          </div>
          <div className="min-w-0">
            <p className="font-display text-base font-semibold leading-tight text-ink-strong">
              Mahal
            </p>
            <p className="font-body text-[11px] text-ink-faint">Decision OS</p>
          </div>
        </Link>

        <button
          type="button"
          onClick={() =>
            onTeaser({
              label: "Location switcher",
              icon: Building2,
              teaser: "Real portfolio and branch switching requires the tenant and membership contracts.",
              status: "gated",
            })
          }
          className="mt-4 flex w-full items-center gap-2 rounded-(--r-md) border border-hairline bg-sunken px-3 py-2 text-left hover:border-rule"
        >
          <Building2 className="h-4 w-4 text-accent-muted" />
          <span className="min-w-0 flex-1">
            <span className="block truncate font-body text-xs font-semibold text-ink-strong">
              Portfolio lens
            </span>
            <span className="block truncate font-body text-[11px] text-ink-faint">
              Tenant context pending
            </span>
          </span>
          <ChevronDown className="h-3.5 w-3.5 text-ink-faint" />
        </button>
      </div>

      <nav className="flex-1 overflow-y-auto px-3 py-4" aria-label="Primary navigation">
        <div className="space-y-5">
          {navGroups.map((group) => (
            <section key={group.label}>
              <p
                className={cn(
                  "mb-2 px-3 font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint",
                  group.label === "Grow" && "text-accent-muted"
                )}
              >
                {group.label}
              </p>
              <div className="space-y-1">
                {group.items.map((item) => (
                  <SidebarItem
                    key={`${group.label}-${item.label}`}
                    item={item}
                    active={!!item.href && (pathname === item.href || pathname.startsWith(`${item.href}/`))}
                    onTeaser={onTeaser}
                  />
                ))}
              </div>
            </section>
          ))}
        </div>
      </nav>
    </aside>
  );
}

function TopBar({ onTeaser }: { onTeaser: (item: NavItem) => void }) {
  const { user, logout } = useAuth();
  const router = useRouter();

  async function handleLogout() {
    await logout();
    router.push("/");
  }

  return (
    <header className="flex h-[var(--topbar-height)] shrink-0 items-center justify-between gap-3 border-b border-hairline bg-page/95 px-4 backdrop-blur lg:px-6">
      <div className="min-w-0">
        <p className="font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint">
          Premium Frontend Foundation
        </p>
        <p className="truncate font-body text-sm text-ink-muted">
          Recommendation first. Evidence second. Actions always visible.
        </p>
      </div>
      <div className="flex items-center gap-2">
        <button
          type="button"
          onClick={() =>
            onTeaser({
              label: "Notifications",
              icon: Bell,
              teaser: "Notification routing waits for operational events and tenant scope.",
              status: "locked",
            })
          }
          className="hidden h-9 w-9 items-center justify-center rounded-(--r-md) border border-hairline bg-surface text-ink-muted hover:bg-raised hover:text-ink-body sm:flex"
          aria-label="Notifications"
        >
          <Bell className="h-4 w-4" />
        </button>
        <Link
          href="/account"
          className="hidden max-w-48 truncate rounded-(--r-md) border border-hairline bg-surface px-3 py-2 font-body text-xs text-ink-muted hover:bg-raised hover:text-ink-body md:block"
          title={user?.email ?? "Account"}
        >
          {user?.email ?? "Account"}
        </Link>
        <button
          type="button"
          onClick={handleLogout}
          className="rounded-(--r-md) border border-hairline bg-surface px-3 py-2 font-body text-xs text-ink-muted hover:bg-raised hover:text-ink-body"
        >
          Sign out
        </button>
      </div>
    </header>
  );
}

function MobileTabBar({ onTeaser }: { onTeaser: (item: NavItem) => void }) {
  const pathname = usePathname();
  const linkClass = (active: boolean) =>
    cn(
      "flex flex-col items-center gap-1 rounded-(--r-md) px-2 py-1.5 font-body text-[10px]",
      active ? "text-accent-muted" : "text-ink-faint"
    );

  return (
    <nav className="fixed inset-x-0 bottom-0 z-40 grid h-[var(--mobile-tabbar-height)] grid-cols-5 border-t border-hairline bg-surface/98 px-2 py-1 lg:hidden">
      <Link href="/dashboard" className={linkClass(pathname.startsWith("/dashboard"))}>
        <Home className="h-4 w-4" />
        Home
      </Link>
      <button
        type="button"
        className={linkClass(false)}
        onClick={() =>
          onTeaser({
            label: "Operate",
            icon: CircleDollarSign,
            status: "locked",
            teaser: "Operate unlocks after tenant-scoped sales, stock, and spend APIs exist.",
          })
        }
      >
        <CircleDollarSign className="h-4 w-4" />
        Operate
      </button>
      <Link href="/workspace" className={linkClass(pathname.startsWith("/workspace"))}>
        <Map className="h-4 w-4" />
        Grow
      </Link>
      <Link href="/compare" className={linkClass(pathname.startsWith("/compare"))}>
        <Layers3 className="h-4 w-4" />
        Compare
      </Link>
      <button
        type="button"
        className={linkClass(false)}
        onClick={() =>
          onTeaser({
            label: "More",
            icon: Settings,
            status: "gated",
            teaser: "Reports, devices, team, and settings remain visible in the desktop shell; mobile launcher follows the same locked-state rules.",
          })
        }
      >
        <Settings className="h-4 w-4" />
        More
      </button>
    </nav>
  );
}

function TeaserPanel({
  item,
  onClose,
}: {
  item: NavItem | null;
  onClose: () => void;
}) {
  if (!item) return null;
  const Icon = item.icon;

  return (
    <div className="fixed bottom-20 left-4 right-4 z-50 rounded-(--r-lg) border border-hairline bg-surface p-4 shadow-[var(--shadow-pop)] lg:bottom-6 lg:left-auto lg:right-6 lg:w-96">
      <div className="flex items-start gap-3">
        <div className="flex h-9 w-9 shrink-0 items-center justify-center rounded-(--r-md) border border-hairline bg-sunken text-accent-muted">
          <Icon className="h-4 w-4" />
        </div>
        <div className="min-w-0 flex-1">
          <p className="font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint">
            {item.status === "locked" ? "Locked module" : "Gated module"}
          </p>
          <h2 className="mt-1 font-display text-lg text-ink-strong">{item.label}</h2>
          <p className="mt-2 font-body text-sm leading-6 text-ink-muted">
            {item.teaser ?? "This surface is visible in the platform shell, but the underlying contract is not ready yet."}
          </p>
        </div>
      </div>
      <button
        type="button"
        onClick={onClose}
        className="mt-4 w-full rounded-(--r-md) border border-hairline bg-sunken px-3 py-2 font-body text-sm text-ink-muted hover:bg-raised hover:text-ink-body"
      >
        Close
      </button>
    </div>
  );
}

export function AppShell({ children }: { children: ReactNode }) {
  const [teaser, setTeaser] = useState<NavItem | null>(null);

  return (
    <div className="flex h-screen overflow-hidden bg-page text-ink-body">
      <Sidebar onTeaser={setTeaser} />
      <div className="flex min-w-0 flex-1 flex-col">
        <TopBar onTeaser={setTeaser} />
        <main className="flex-1 overflow-y-auto pb-[var(--mobile-tabbar-height)] lg:pb-0">
          <AuthGuard>{children}</AuthGuard>
        </main>
      </div>
      <MobileTabBar onTeaser={setTeaser} />
      <TeaserPanel item={teaser} onClose={() => setTeaser(null)} />
    </div>
  );
}
