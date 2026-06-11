import { cn } from "@/lib/utils";

interface StatProps {
  label: string;
  value: string | number;
  sub?: string;
  trend?: "up" | "down" | "flat";
  className?: string;
}

const trendIcons = {
  up:   { symbol: "↑", color: "text-verdict-strong" },
  down: { symbol: "↓", color: "text-verdict-weak" },
  flat: { symbol: "→", color: "text-ink-muted" },
};

export function Stat({ label, value, sub, trend, className }: StatProps) {
  return (
    <div className={cn("flex flex-col gap-0.5", className)}>
      <p className="font-body text-xs uppercase tracking-widest text-ink-faint">
        {label}
      </p>
      <div className="flex items-baseline gap-1.5">
        <span className="font-mono text-2xl font-medium text-ink-strong leading-none">
          {value}
        </span>
        {trend && (
          <span className={cn("font-mono text-sm leading-none", trendIcons[trend].color)}>
            {trendIcons[trend].symbol}
          </span>
        )}
      </div>
      {sub && (
        <p className="font-body text-xs text-ink-muted">{sub}</p>
      )}
    </div>
  );
}
