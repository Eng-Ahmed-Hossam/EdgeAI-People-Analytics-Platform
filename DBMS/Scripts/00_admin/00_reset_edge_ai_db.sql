-- ==========================================
-- reset_edge_ai_db.sql
-- HARD RESET (USE WITH CAUTION)
-- ==========================================

DO $$
BEGIN
    RAISE NOTICE '⚠️ DROPPING Edge AI Location Intelligence database objects';
END $$;

DROP SCHEMA IF EXISTS public CASCADE;
CREATE SCHEMA public;

GRANT ALL ON SCHEMA public TO postgres;
GRANT ALL ON SCHEMA public TO public;

DO $$
BEGIN
    RAISE NOTICE '✅ Database schema reset completed';
END $$;
