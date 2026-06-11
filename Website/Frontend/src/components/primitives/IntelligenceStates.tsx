import type { ReactNode } from "react";
import { cn } from "@/lib/utils";

interface IntelligenceStateProps {
  kicker: string;
  title: string;
  summary: string;
  children?: ReactNode;
  className?: string;
}

export function IntelligenceEmptyState({
  kicker,
  title,
  summary,
  children,
  className,
}: IntelligenceStateProps) {
  return (
    <section
      className={cn(
        "rounded-(--r-lg) border border-hairline bg-surface p-6 shadow-[var(--shadow-card)]",
        className
      )}
    >
      <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
        {kicker}
      </p>
      <h1 className="mt-2 font-display text-2xl text-ink-strong">{title}</h1>
      <p className="mt-3 max-w-2xl font-body text-sm leading-6 text-ink-muted">
        {summary}
      </p>
      {children && <div className="mt-5">{children}</div>}
    </section>
  );
}

export function IntelligenceErrorState({
  kicker,
  title,
  summary,
  children,
  className,
}: IntelligenceStateProps) {
  return (
    <section
      className={cn(
        "rounded-(--r-lg) border border-verdict-weak/40 bg-verdict-weak-wash p-6",
        className
      )}
    >
      <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
        {kicker}
      </p>
      <h1 className="mt-2 font-display text-2xl text-ink-strong">{title}</h1>
      <p className="mt-3 max-w-2xl font-body text-sm leading-6 text-ink-body">
        {summary}
      </p>
      {children && <div className="mt-5">{children}</div>}
    </section>
  );
}

export function IntelligenceLoadingState({
  kicker,
  title,
  summary,
}: Omit<IntelligenceStateProps, "children" | "className">) {
  return (
    <section className="rounded-(--r-lg) border border-hairline bg-surface p-6 shadow-[var(--shadow-card)]">
      <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
        {kicker}
      </p>
      <h1 className="mt-2 font-display text-2xl text-ink-strong">{title}</h1>
      <p className="mt-3 max-w-2xl font-body text-sm leading-6 text-ink-muted">
        {summary}
      </p>

      <div className="mt-6 space-y-3">
        <div className="h-3 w-2/3 animate-pulse rounded-full bg-raised" />
        <div className="h-3 w-1/2 animate-pulse rounded-full bg-raised" />
        <div className="grid gap-3 pt-2 sm:grid-cols-3">
          {[0, 1, 2].map((item) => (
            <div
              className="h-24 animate-pulse rounded-(--r-md) border border-hairline bg-sunken"
              key={item}
            />
          ))}
        </div>
      </div>
    </section>
  );
}
