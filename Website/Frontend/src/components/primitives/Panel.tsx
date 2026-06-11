import { cn } from "@/lib/utils";
import { HTMLAttributes, ReactNode } from "react";

interface PanelProps extends HTMLAttributes<HTMLDivElement> {
  header?: ReactNode;
  footer?: ReactNode;
  width?: string;
}

export function Panel({ header, footer, width = "w-[440px]", className, children, ...props }: PanelProps) {
  return (
    <div
      className={cn(
        "flex flex-col bg-surface border-l border-hairline h-full",
        width,
        "max-w-full shrink-0",
        className
      )}
      {...props}
    >
      {header && (
        <div className="shrink-0 border-b border-hairline">
          {header}
        </div>
      )}
      <div className="flex-1 overflow-y-auto overscroll-contain">
        {children}
      </div>
      {footer && (
        <div className="shrink-0 border-t border-hairline">
          {footer}
        </div>
      )}
    </div>
  );
}

interface PanelSectionProps extends HTMLAttributes<HTMLDivElement> {
  label?: string;
  actions?: ReactNode;
}

export function PanelSection({ label, actions, className, children, ...props }: PanelSectionProps) {
  return (
    <div className={cn("py-5", className)} {...props}>
      {(label || actions) && (
        <div className="flex items-center justify-between px-6 mb-3">
          {label && (
            <h3 className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
              {label}
            </h3>
          )}
          {actions}
        </div>
      )}
      <div className={cn(!(label || actions) && "px-6")}>{children}</div>
    </div>
  );
}
