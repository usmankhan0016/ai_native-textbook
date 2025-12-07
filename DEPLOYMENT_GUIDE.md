# Deployment Guide

## Landing Page Redesign - Production Deployment

### Current Status
- ✅ Backup branch: `landing-page-redesign` (pushed to GitHub)
- ✅ Build verified: No errors
- ⏳ PR pending: Manual creation on GitHub

---

## Deployment Options

### Option 1: GitHub Pages (Free, Recommended)

**Steps:**
1. Merge PR to `master`
2. Enable GitHub Pages in repo settings
3. Set source to `/ (root)` on `master` branch
4. Build: Automatic (GitHub Actions)
5. Live at: `https://usmankhan0016.github.io/ai_native-textbook`

**Setup time:** 2 minutes

---

### Option 2: Vercel (Recommended for Speed)

**Steps:**
1. Go to https://vercel.com
2. Import repository
3. Set build: `npm run build`
4. Deploy
5. Live at: `yourdomain.vercel.app`

**Benefits:** Instant deployment, auto-preview on PRs

**Setup time:** 1 minute

---

### Option 3: Self-Hosted / Custom Domain

Requires:
- Web server (Nginx, Apache)
- Static files from `build/` folder
- Custom domain configured

---

## Quick Merge & Deploy (5 minutes)

```bash
# 1. Create PR on GitHub (if not done)
# https://github.com/usmankhan0016/ai_native-textbook

# 2. Merge PR (click "Merge pull request" button)

# 3. Pull to local
git checkout master
git pull origin master

# 4. Verify build
npm run build

# 5. Deploy (choose one):
#    - GitHub Pages: Enable in Settings
#    - Vercel: Run `npm i -g vercel && vercel`
#    - Custom: Upload `build/` folder
```

---

## Production Checklist

- [x] Build passes without errors
- [x] Responsive design verified
- [x] Dark mode tested
- [x] Textbook content protected
- [x] Logo and assets included
- [ ] Create PR on GitHub
- [ ] Merge to master
- [ ] Deploy to production
- [ ] Test live URL

---

## Rollback

If needed, revert to backup:
```bash
git checkout landing-page-redesign  # Original redesign
git log --oneline                   # See commit history
git revert <commit-hash>            # Revert specific change
```

---

## Support

Questions? Check:
- Docusaurus docs: https://docusaurus.io/docs
- GitHub Pages: https://docs.github.com/en/pages
- Vercel deployment: https://vercel.com/docs
