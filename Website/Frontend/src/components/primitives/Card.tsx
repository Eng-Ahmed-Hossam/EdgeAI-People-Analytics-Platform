import { cn } from "@/lib/utils";
import { HTMLAttributes } from "react";

interface CardProps extends HTMLAttributes<HTMLDivElement> {
  elevated?: boolean;
}

export function Card({ elevated = false, className, children, ...props }: CardProps) {
  return (
    <div
      className={cn(
        "bg-surface border border-hairline rounded-(--r-lg)",
        elevated ? "shadow-[var(--shadow-pop)]" : "shadow-[var(--shadow-card)]",
        className
      )}
      {...props}
    >
      {children}
    </div>
  );
}

interface CardSectionProps extends HTMLAttributes<HTMLDivElement> {
  flush?: boolean;
}

export function CardSection({ flush = false, className, children, ...props }: CardSectionProps) {
  return (
    <div
      className={cn(!flush && "px-6 py-5", className)}
      {...props}
    >
      {children}
    </div>
  );
}

export function CardDivider({ className }: { className?: string }) {
  return <div className={cn("h-px bg-hairline", className)} />;
}

interface CardHeaderProps extends HTMLAttributes<HTMLDivElement> {
  label?: string;
}

export function CardHeader({ label, className, children, ...props }: CardHeaderProps) {
  return (
    <div
      className={cn(
        "flex items-center justify-between px-6 py-3 bg-sunken border-b border-hairline rounded-t-(--r-lg)",
        className
      )}
      {...props}
    >
      {label && (
        <span className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
          {label}
        </span>
      )}
      {children}
    </div>
  );
}
