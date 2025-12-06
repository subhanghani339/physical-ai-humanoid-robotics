# Urdu Translation Feature Specification

## Project Goal
Add dynamic Urdu translation capability to the Physical AI textbook with a button at the start of each chapter that translates content on-demand using OpenAI API.

## Requirements (50 Bonus Points)
- Translation button at the START of each chapter
- Toggle between English and Urdu
- Right-to-left (RTL) text direction for Urdu
- Proper Urdu fonts (Noto Nastalikh Urdu)
- Cache translations to avoid re-translating
- Preserve code blocks, technical terms, and formatting
- Backend API endpoint for translation
- Smooth user experience with loading states

## Technical Architecture

### Frontend Component (React + TypeScript)
Location: `src/components/TranslateButton.tsx`

Component Requirements:
1. Accept chapter content as prop
2. Display button with Urdu and English flags/text
3. Show loading spinner during translation
4. Toggle between original and translated content
5. Cache translated content in component state
6. Handle errors gracefully
7. Responsive design (works on mobile)

Button States:
- Initial: "🇵🇰 اردو میں پڑھیں" (Read in Urdu)
- Loading: "🔄 ترجمہ ہو رہا ہے..." (Translating...)
- Translated: "🇬🇧 Show English"
- Error: Show error message

### Styling
Location: `src/components/TranslateButton.module.css`

Requirements:
- Professional gradient button (purple/blue)
- Hover effects and animations
- RTL container for Urdu text
- Proper spacing and padding
- Dark mode compatible
- Mobile responsive

### Urdu Font Setup
Location: `src/css/custom.css`

Requirements:
- Import Google Fonts: 'Noto Nastalikh Urdu'
- Set font-family for RTL content
- Proper line-height for Urdu readability (1.8-2.0)
- Keep code blocks in LTR with monospace font
- Larger font size for Urdu (18-20px)

### Backend API Endpoint
Location: `backend/translation.py`

Endpoint: `POST /api/translate`

Request Body:
```json